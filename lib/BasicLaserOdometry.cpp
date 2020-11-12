#include "loam_velodyne/BasicLaserOdometry.h"
#include "loam_velodyne/math_utils.h"
#include <pcl/filters/filter.h>
#include <Eigen/Eigenvalues>
#include <Eigen/QR>

namespace loam
{

using std::sin;
using std::cos;
using std::asin;
using std::atan2;
using std::sqrt;
using std::fabs;
using std::pow;


BasicLaserOdometry::BasicLaserOdometry(float scanPeriod, size_t maxIterations) :
   _scanPeriod(scanPeriod),
   _systemInited(false),
   _frameCount(0),
   _maxIterations(maxIterations),
   _deltaTAbort(0.1),
   _deltaRAbort(0.1),
   _cornerPointsSharp(new pcl::PointCloud<pcl::PointXYZI>()),
   _cornerPointsLessSharp(new pcl::PointCloud<pcl::PointXYZI>()),
   _surfPointsFlat(new pcl::PointCloud<pcl::PointXYZI>()),
   _surfPointsLessFlat(new pcl::PointCloud<pcl::PointXYZI>()),
   _laserCloud(new pcl::PointCloud<pcl::PointXYZI>()),
   _lastCornerCloud(new pcl::PointCloud<pcl::PointXYZI>()),
   _lastSurfaceCloud(new pcl::PointCloud<pcl::PointXYZI>()),
   _laserCloudOri(new pcl::PointCloud<pcl::PointXYZI>()),
   _coeffSel(new pcl::PointCloud<pcl::PointXYZI>())
{}


//当前点云中的点相对第一个点去除因匀速运动产生的畸变，效果相当于得到在点云扫描开始位置静止扫描得到的点云
void BasicLaserOdometry::transformToStart(const pcl::PointXYZI& pi, pcl::PointXYZI& po)
{
   //插值系数计算
   float s = (1.f / _scanPeriod) * (pi.intensity - int(pi.intensity));

   po.x = pi.x - s * _transform.pos.x();
   po.y = pi.y - s * _transform.pos.y();
   po.z = pi.z - s * _transform.pos.z();
   po.intensity = pi.intensity;

   Angle rx = -s * _transform.rot_x.rad();
   Angle ry = -s * _transform.rot_y.rad();
   Angle rz = -s * _transform.rot_z.rad();
   rotateZXY(po, rz, rx, ry);
}


//将上一帧点云中的点相对结束位置去除因匀速运动产生的畸变，效果相当于得到在点云扫描结束位置静止扫描得到的点云
size_t BasicLaserOdometry::transformToEnd(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
   size_t cloudSize = cloud->points.size();

   for (size_t i = 0; i < cloudSize; i++)
   {
      pcl::PointXYZI& point = cloud->points[i];

      float s = (1.f / _scanPeriod) * (point.intensity - int(point.intensity));

      //这里都是减号，是因为通过优化计算出来的变换是从当前点云帧结束时刻到初始时刻的
      //而将点云全部投影到结束时刻则需要加个负号
      point.x -= s * _transform.pos.x();
      point.y -= s * _transform.pos.y();
      point.z -= s * _transform.pos.z();
      point.intensity = int(point.intensity);

      Angle rx = -s * _transform.rot_x.rad();
      Angle ry = -s * _transform.rot_y.rad();
      Angle rz = -s * _transform.rot_z.rad();
      rotateZXY(point, rz, rx, ry);
      rotateYXZ(point, _transform.rot_y, _transform.rot_x, _transform.rot_z);

      point.x += _transform.pos.x() - _imuShiftFromStart.x();
      point.y += _transform.pos.y() - _imuShiftFromStart.y();
      point.z += _transform.pos.z() - _imuShiftFromStart.z();

      rotateZXY(point, _imuRollStart, _imuPitchStart, _imuYawStart);
      rotateYXZ(point, -_imuYawEnd, -_imuPitchEnd, -_imuRollEnd);
   }

   return cloudSize;
}


//LOAM算法定义了激光雷达坐标系L世界坐标系W，W与L的初始位姿重合，可理解为W与激光雷达输出的第一个点云的坐标系重合
//该函数计算当前点云帧结束时相对于第一个点云即世界坐标系W的累积旋转量，R=Rtransformsum*Rtransform
//注意，是旋转矩阵相乘，不是欧拉角直接相加，因此，下面的计算也是通过将欧拉角转换为旋转矩阵，相乘之后再反过来计算欧拉角
void BasicLaserOdometry::accumulateRotation(Angle cx, Angle cy, Angle cz,
                                            Angle lx, Angle ly, Angle lz,
                                            Angle &ox, Angle &oy, Angle &oz)
{
   //cx,cy,cz是从第一帧到上一帧点云累计的旋转量的欧拉角
   //lx,ly,lz是当前帧从初始时刻到结束时刻旋转量的欧拉角
   //此处欧拉角到旋转矩阵的转换遵循绕YXZ的顺序，旋转矩阵表示如下(参考维基百科：https://en.wikipedia.org/wiki/Euler_angles ,5.1Rotation Matrix)
   //|sΘx*sΘy*sΘz+cΘy*cΘz , sΘx*sΘy*cΘz-cΘy*sΘz , cΘx*sΘy|
   //|     cΘx*sΘz        ,        cΘx*cΘz      ,  -sΘx  |
   //|sΘx*cΘy*sΘz-sΘy*cΘz , sΘx*cΘy*cΘz+sΘy*sΘz , cΘy*cΘx|
   //将Rtransformsum和Rtransform转变为上述旋转矩阵表示，根据R=Rtransformsum*Rtransform(R=Rtfs*Rtf)各个元素相互对应
   //有R(2,3)=Rtfs(2,1)*Rtf(1,3)+Rtfs(2,2)*Rtf(2,3)+Rtfs(2,3)*Rtf(3,3)=-sΘx，
   float srx = lx.cos()*cx.cos()*ly.sin()*cz.sin()
      - cx.cos()*cz.cos()*lx.sin()
      - lx.cos()*ly.cos()*cx.sin();
   //即当前点云帧结束时刻激光雷达位姿相对于世界坐标系W的累积旋转量的欧拉角表示的X轴分量为Θx=-asinR(2,3)
   ox = -asin(srx);

   //有R(1,3)=Rtfs(1,2)*Rtf(2,3)+Rtfs(1,1)*Rtf(1,3)+Rtfs(1,3)*Rtf(3,3)=cΘx*sΘy
   float srycrx = lx.sin()*(cy.cos()*cz.sin() - cz.cos()*cx.sin()*cy.sin())
      + lx.cos()*ly.sin()*(cy.cos()*cz.cos() + cx.sin()*cy.sin()*cz.sin())
      + lx.cos()*ly.cos()*cx.cos()*cy.sin();
   //有R(3,3)=Rtfs(3,3)*Rtf(3,3)+Rtfs(3,1)*Rtf(1,3)+Rtfs(3,2)*Rtf(2,3)=cΘy*cΘx
   float crycrx = lx.cos()*ly.cos()*cx.cos()*cy.cos()
      - lx.cos()*ly.sin()*(cz.cos()*cy.sin() - cy.cos()*cx.sin()*cz.sin())
      - lx.sin()*(cy.sin()*cz.sin() + cy.cos()*cz.cos()*cx.sin());
   //Y轴分量Θy=atan2(R(1,3)/cΘx,R(3,3)/cΘx)
   oy = atan2(srycrx / ox.cos(), crycrx / ox.cos());

   //有R(2,1)=Rtfs(2,3)*Rtf(3,1)+Rtfs(2,1)*Rtf(1,1)+Rtfs(2,2)*Rtf(2,1)=cΘx*sΘz
   float srzcrx = cx.sin()*(lz.cos()*ly.sin() - ly.cos()*lx.sin()*lz.sin())
      + cx.cos()*cz.sin()*(ly.cos()*lz.cos() + lx.sin()*ly.sin()*lz.sin())
      + lx.cos()*cx.cos()*cz.cos()*lz.sin();
   //有R(2,2)=Rtfs(2,2)*Rtf(2,2)+Rtfs(2,1)*Rtf(1,2)+Rtfs(2,3)*Rtf(3,2)=cΘx*cΘz
   float crzcrx = lx.cos()*lz.cos()*cx.cos()*cz.cos()
      - cx.cos()*cz.sin()*(ly.cos()*lz.sin() - lz.cos()*lx.sin()*ly.sin())
      - cx.sin()*(ly.sin()*lz.sin() + ly.cos()*lz.cos()*lx.sin());
   //Z轴分量Θz=atan2(R(2,1)/cΘx,R(2,2)/cΘx)
   oz = atan2(srzcrx / ox.cos(), crzcrx / ox.cos());
}


//利用IMU修正旋转量，根据起始欧拉角，当前点云的欧拉角修正，具体的实施过程和BasicLaserOdometry::accumulateRotation()是一样的
void BasicLaserOdometry::pluginIMURotation(const Angle& bcx, const Angle& bcy, const Angle& bcz,
                                           const Angle& blx, const Angle& bly, const Angle& blz,
                                           const Angle& alx, const Angle& aly, const Angle& alz,
                                           Angle &acx, Angle &acy, Angle &acz)
{
/*
   //根据IMU修正旋转量
   pluginIMURotation(rx, ry, rz,
                     _imuPitchStart, _imuYawStart, _imuRollStart,
                     _imuPitchEnd, _imuYawEnd, _imuRollEnd,
                     rx, ry, rz);
   
*/

   //全局欧拉角，即当前点云帧结束时刻的位姿相对于世界坐标系的欧拉角
   float sbcx = bcx.sin();
   float cbcx = bcx.cos();
   float sbcy = bcy.sin();
   float cbcy = bcy.cos();
   float sbcz = bcz.sin();
   float cbcz = bcz.cos();

   //当前点云帧初始时刻的IMU欧拉角
   float sblx = blx.sin();
   float cblx = blx.cos();
   float sbly = bly.sin();
   float cbly = bly.cos();
   float sblz = blz.sin();
   float cblz = blz.cos();

   //当前点云帧结束时刻的IMU欧拉角
   float salx = alx.sin();
   float calx = alx.cos();
   float saly = aly.sin();
   float caly = aly.cos();
   float salz = alz.sin();
   float calz = alz.cos();
   //|sΘx*sΘy*sΘz+cΘy*cΘz , sΘx*sΘy*cΘz-cΘy*sΘz , cΘx*sΘy|
   //|     cΘx*sΘz        ,        cΘx*cΘz      ,  -sΘx  |
   //|sΘx*cΘy*sΘz-sΘy*cΘz , sΘx*cΘy*cΘz+sΘy*sΘz , cΘy*cΘx|
   //这两个IMU的欧拉角是在世界坐标系下的全局欧拉角，和上面一个根据点云优化迭代计算出来的全局欧拉角是不一样的
   //其处理过程在BasicScanRegistration::updateIMUTransform()

   //以下的计算同BasicLaserOdometry::accumulateRotation()，区别在于这里是三个旋转矩阵相乘
   //R=全局欧拉角Rbc*初始时刻欧拉角的转置RblT*结束时刻欧拉角Ral
   //参考论文low-drift and real-time lidar odometry and mapping"中给出的解释是让当前帧点云对齐该帧初始时刻的方向

   float srx = -sbcx * (salx*sblx + calx * caly*cblx*cbly + calx * cblx*saly*sbly)
      - cbcx * cbcz*(calx*saly*(cbly*sblz - cblz * sblx*sbly)
                     - calx * caly*(sbly*sblz + cbly * cblz*sblx) + cblx * cblz*salx)
      - cbcx * sbcz*(calx*caly*(cblz*sbly - cbly * sblx*sblz)
                     - calx * saly*(cbly*cblz + sblx * sbly*sblz) + cblx * salx*sblz);
   acx = -asin(srx);

   float srycrx = (cbcy*sbcz - cbcz * sbcx*sbcy)*(calx*saly*(cbly*sblz - cblz * sblx*sbly)
                                                  - calx * caly*(sbly*sblz + cbly * cblz*sblx) + cblx * cblz*salx)
      - (cbcy*cbcz + sbcx * sbcy*sbcz)*(calx*caly*(cblz*sbly - cbly * sblx*sblz)
                                        - calx * saly*(cbly*cblz + sblx * sbly*sblz) + cblx * salx*sblz)
      + cbcx * sbcy*(salx*sblx + calx * caly*cblx*cbly + calx * cblx*saly*sbly);
   float crycrx = (cbcz*sbcy - cbcy * sbcx*sbcz)*(calx*caly*(cblz*sbly - cbly * sblx*sblz)
                                                  - calx * saly*(cbly*cblz + sblx * sbly*sblz) + cblx * salx*sblz)
      - (sbcy*sbcz + cbcy * cbcz*sbcx)*(calx*saly*(cbly*sblz - cblz * sblx*sbly)
                                        - calx * caly*(sbly*sblz + cbly * cblz*sblx) + cblx * cblz*salx)
      + cbcx * cbcy*(salx*sblx + calx * caly*cblx*cbly + calx * cblx*saly*sbly);
   acy = atan2(srycrx / acx.cos(), crycrx / acx.cos());

   float srzcrx = sbcx * (cblx*cbly*(calz*saly - caly * salx*salz) - cblx * sbly*(caly*calz + salx * saly*salz) + calx * salz*sblx)
      - cbcx * cbcz*((caly*calz + salx * saly*salz)*(cbly*sblz - cblz * sblx*sbly)
                     + (calz*saly - caly * salx*salz)*(sbly*sblz + cbly * cblz*sblx)
                     - calx * cblx*cblz*salz)
      + cbcx * sbcz*((caly*calz + salx * saly*salz)*(cbly*cblz + sblx * sbly*sblz)
                     + (calz*saly - caly * salx*salz)*(cblz*sbly - cbly * sblx*sblz)
                     + calx * cblx*salz*sblz);
   float crzcrx = sbcx * (cblx*sbly*(caly*salz - calz * salx*saly) - cblx * cbly*(saly*salz + caly * calz*salx) + calx * calz*sblx)
      + cbcx * cbcz*((saly*salz + caly * calz*salx)*(sbly*sblz + cbly * cblz*sblx)
                     + (caly*salz - calz * salx*saly)*(cbly*sblz - cblz * sblx*sbly)
                     + calx * calz*cblx*cblz)
      - cbcx * sbcz*((saly*salz + caly * calz*salx)*(cblz*sbly - cbly * sblx*sblz)
                     + (caly*salz - calz * salx*saly)*(cbly*cblz + sblx * sbly*sblz)
                     - calx * calz*cblx*sblz);
   acz = atan2(srzcrx / acx.cos(), crzcrx / acx.cos());
}


//被LaserOdometry::imuTransHandler调用，接收带有IMU数据的topic并提取IMU信息
void BasicLaserOdometry::updateIMU(pcl::PointCloud<pcl::PointXYZ> const& imuTrans)
{
   assert(4 == imuTrans.size());
   _imuPitchStart = imuTrans.points[0].x;
   _imuYawStart = imuTrans.points[0].y;
   _imuRollStart = imuTrans.points[0].z;

   _imuPitchEnd = imuTrans.points[1].x;
   _imuYawEnd = imuTrans.points[1].y;
   _imuRollEnd = imuTrans.points[1].z;

   _imuShiftFromStart = imuTrans.points[2];
   _imuVeloFromStart = imuTrans.points[3];
}

void BasicLaserOdometry::process()
{
   if (!_systemInited)
   {//运动估计需要前后两帧点云，刚收到第一帧点云时先不进行处理，得到收到第二帧再进行处理
      //并保证上一次的点云_lastCornerCloud存储的是上一帧点云中曲率较大的点云，即带有less的点云
      _cornerPointsLessSharp.swap(_lastCornerCloud);
      _surfPointsLessFlat.swap(_lastSurfaceCloud);

      //使用上一帧的点云特征点构建kd-tree，方便查找最近点
      _lastCornerKDTree.setInputCloud(_lastCornerCloud);
      _lastSurfaceKDTree.setInputCloud(_lastSurfaceCloud);

      _transformSum.rot_x += _imuPitchStart;
      _transformSum.rot_z += _imuRollStart;

      _systemInited = true;
      return;
   }

   pcl::PointXYZI coeff;
   bool isDegenerate = false;//退化标志
   Eigen::Matrix<float, 6, 6> matP;//P矩阵，预测矩阵

   _frameCount++;
   //T平移量的初值赋值为加减速的位移量，为其梯度下降的方向
   //（沿用上次转换的T（一个sweep匀速模型），同时在其基础上减去匀速运动位移，即只考虑加减速的位移量）
   _transform.pos -= _imuVeloFromStart * _scanPeriod;


   size_t lastCornerCloudSize = _lastCornerCloud->points.size();
   size_t lastSurfaceCloudSize = _lastSurfaceCloud->points.size();

   if (lastCornerCloudSize > 10 && lastSurfaceCloudSize > 100)
   {
      std::vector<int> pointSearchInd(1);
      std::vector<float> pointSearchSqDis(1);
      std::vector<int> indices;

      pcl::removeNaNFromPointCloud(*_cornerPointsSharp, *_cornerPointsSharp, indices);
      size_t cornerPointsSharpNum = _cornerPointsSharp->points.size();
      size_t surfPointsFlatNum = _surfPointsFlat->points.size();

      _pointSearchCornerInd1.resize(cornerPointsSharpNum);
      _pointSearchCornerInd2.resize(cornerPointsSharpNum);
      _pointSearchSurfInd1.resize(surfPointsFlatNum);
      _pointSearchSurfInd2.resize(surfPointsFlatNum);
      _pointSearchSurfInd3.resize(surfPointsFlatNum);

      //Levenberg-Marquardt算法(L-M method)，非线性最小二乘算法，最优化算法的一种
      //最多迭代25次
      for (size_t iterCount = 0; iterCount < _maxIterations; iterCount++)
      {
         pcl::PointXYZI pointSel, pointProj, tripod1, tripod2, tripod3;
         _laserCloudOri->clear();//每次迭代前清空上一次迭代中存储的被选中的上一帧点云的点
         _coeffSel->clear();

         //处理edge point，寻找上一帧点云中与之最近的且能构成直线的两点
         //处理当前点云中的曲率最大的特征点,从上个点云中曲率比较大的特征点中找两个最近距离点，
         //一个点使用kd-tree查找，另一个根据找到的点在其相邻线找另外一个最近距离的点
         for (int i = 0; i < cornerPointsSharpNum; i++)
         {
            //此处没有直接将_cornerPointsSharp中的点投影到该帧点云的初始时刻，而是透过pointSel变量
            //是因为后续建立优化方程还需要用到_surfPointsFlat中的特征点
            transformToStart(_cornerPointsSharp->points[i], pointSel);

            if (iterCount % 5 == 0)
            {//每迭代五次，重新查找最近点
               //kd-tree查找一个最近距离点，边沿点未经过体素栅格滤波，一般边沿点本来就比较少，不做滤波
               //pointSearchInd——最近点的序号，pointSearchSqDis——离最近点的距离
               pcl::removeNaNFromPointCloud(*_lastCornerCloud, *_lastCornerCloud, indices);
               _lastCornerKDTree.nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
               int closestPointInd = -1, minPointInd2 = -1;

               //寻找相邻线距离目标点距离最小的点，这里没有再采用kdTree.nearestKSearch()查找，而是遍历临近scan的点
               //原因是要确保这两个点能构成合理的直线
               //再次提醒：velodyne是2度一线，scanID相邻并不代表线号相邻，相邻线度数相差2度，也即线号scanID相差2
               if (pointSearchSqDis[0] < 25)//找到的最近点距离的确很近的话
               {
                  //提取最近点线号
                  closestPointInd = pointSearchInd[0];
                  int closestPointScan = int(_lastCornerCloud->points[closestPointInd].intensity);

                  float pointSqDis, minPointSqDis2 = 25;//初始门槛值5米，可大致过滤掉scanID相邻，但实际线不相邻的值
                  for (int j = closestPointInd + 1; j < cornerPointsSharpNum; j++)
                  {//向scanID增大的方向查找，实际是按照点云中点的序号朝增大的方向查找，同时不能超过点云中点的数量
                     if (int(_lastCornerCloud->points[j].intensity) > closestPointScan + 2.5)
                     {//非相邻线
                        break;
                     }

                     pointSqDis = calcSquaredDiff(_lastCornerCloud->points[j], pointSel);

                     if (int(_lastCornerCloud->points[j].intensity) > closestPointScan)
                     {//确保两个点不在同一条scan上（相邻线查找应该可以用scanID == closestPointScan +/- 1 来做）
                        if (pointSqDis < minPointSqDis2)
                        {//距离更近，要小于初始值5米
                           //更新最小距离与点序
                           minPointSqDis2 = pointSqDis;
                           minPointInd2 = j;
                        }
                     }
                  }
                  for (int j = closestPointInd - 1; j >= 0; j--)
                  {//向scanID减小的方向查找
                     if (int(_lastCornerCloud->points[j].intensity) < closestPointScan - 2.5)
                     {
                        break;
                     }

                     pointSqDis = calcSquaredDiff(_lastCornerCloud->points[j], pointSel);

                     if (int(_lastCornerCloud->points[j].intensity) < closestPointScan)
                     {
                        if (pointSqDis < minPointSqDis2)
                        {
                           minPointSqDis2 = pointSqDis;
                           minPointInd2 = j;
                        }
                     }
                  }
               }
               //记住组成线的点序
               _pointSearchCornerInd1[i] = closestPointInd;//kd-tree最近距离点，-1表示未找到满足的点
               _pointSearchCornerInd2[i] = minPointInd2;//另一个最近的，-1表示未找到满足的点
            }

            //计算edge point到上一帧点云中与之最近的edge line的距离
            //参考论文"ow-drift and real-time lidar odometry and mapping"
            if (_pointSearchCornerInd2[i] >= 0)
            {//大于等于0，不等于-1，说明两个点都找到了
               tripod1 = _lastCornerCloud->points[_pointSearchCornerInd1[i]];
               tripod2 = _lastCornerCloud->points[_pointSearchCornerInd2[i]];

               //选择的特征点记为O，kd-tree最近距离点记为A，另一个最近距离点记为B
               float x0 = pointSel.x;//O
               float y0 = pointSel.y;
               float z0 = pointSel.z;
               float x1 = tripod1.x;//A
               float y1 = tripod1.y;
               float z1 = tripod1.z;
               float x2 = tripod2.x;//B
               float y2 = tripod2.y;
               float z2 = tripod2.z;

               //公式(2)，求点O到点A、B构成的直线的距离，利用这三点构成的三角形的两种面积公式(几何、向量)求
               //面积A0=1/2*b*h(二分之一的底乘以高)，A0=1/2*OA×OB(向量OA、OB叉乘等于两者构成的平行四边形的面积，三角形面积即为该四边形的一半)
               //向量OA = (x0 - x1, y0 - y1, z0 - z1), 向量OB = (x0 - x2, y0 - y2, z0 - z2)
               //向量AB = (x1 - x2, y1 - y2, z1 - z2)，其模即为三角形的底边长
               //向量OA OB的向量积(即叉乘)为：
               //|  i      j      k  |
               //|x0-x1  y0-y1  z0-z1|
               //|x0-x2  y0-y2  z0-z2|
               //模为：
               float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                                 * ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                                 + ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                                 * ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                                 + ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))
                                 * ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));

               //两个最近距离点之间的距离，即向量AB的模
               float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

               //AB向量与OAB平面的法向量的叉积的单位向量在各轴上的分量
               //即AB×(OA×OB)/|OA×OB|/|AB|，该向量最终表示的是距离ld2方向的单位向量
               //因此利用该方法求点到直线的距离也可解释为求线外一点与线内一点所构成的向量在上述单位向量方向上的投影
               //以下是该单位向量在XYZ坐标轴上的分量
               //x轴分量i
               float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                           + (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;
               //y轴分量j
               float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                            - (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;
               //z轴分量k
               float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                            + (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

               float ld2 = a012 / l12; // Eq. (2)//计算出距离

               // TODO: Why writing to a variable that's never read?
               pointProj = pointSel;
               pointProj.x -= la * ld2;
               pointProj.y -= lb * ld2;
               pointProj.z -= lc * ld2;

               //权重计算，点到直线的距离越大权重越小，距离越小权重越大
               //可以理解为在最小二乘问题中，距离越远，其置信度越低，即该点实际上很有可能不在上一帧点云中对应的直线上
               //下面处理平面点也是出于同样的考虑
               float s = 1;
               if (iterCount >= 5)
               {//5次迭代之后开始增加权重因素
                  s = 1 - 1.8f * fabs(ld2);
               }

               //考虑权重
               coeff.x = s * la;
               coeff.y = s * lb;
               coeff.z = s * lc;
               coeff.intensity = s * ld2;

               if (s > 0.1 && ld2 != 0)
               {//只保留权重大的，也即距离比较小的点，同时也舍弃距离为零的
                  _laserCloudOri->push_back(_cornerPointsSharp->points[i]);
                  _coeffSel->push_back(coeff);
               }
            }
         }

         //处理planar point，
         //对本次接收到的曲率最小的点,从上次接收到的点云曲率比较小的点中找三点组成平面，
         //一个使用kd-tree查找，另外一个在同一线上查找满足要求的，第三个在不同线上查找满足要求的
         //与上面对edge point的处理类似
         for (int i = 0; i < surfPointsFlatNum; i++)
         {
            //此处没有直接将_surfPointsFlat中的点投影到该帧点云的初始时刻，而是透过pointSel变量
            //是因为后续建立优化方程还需要用到_surfPointsFlat中的特征点
            transformToStart(_surfPointsFlat->points[i], pointSel);

            if (iterCount % 5 == 0)
            {
               //kd-tree查找第一个最近点，pointSearchInd——最近点的序号，pointSearchSqDis——离最近点的距离
               _lastSurfaceKDTree.nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
               int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
               if (pointSearchSqDis[0] < 25)
               {
                  closestPointInd = pointSearchInd[0];
                  int closestPointScan = int(_lastSurfaceCloud->points[closestPointInd].intensity);

                  float pointSqDis, minPointSqDis2 = 25, minPointSqDis3 = 25;
                  for (int j = closestPointInd + 1; j < surfPointsFlatNum; j++)
                  {//
                     if (int(_lastSurfaceCloud->points[j].intensity) > closestPointScan + 2.5)
                     {
                        break;
                     }

                     pointSqDis = calcSquaredDiff(_lastSurfaceCloud->points[j], pointSel);

                     if (int(_lastSurfaceCloud->points[j].intensity) <= closestPointScan)
                     {
                        if (pointSqDis < minPointSqDis2)
                        {
                           minPointSqDis2 = pointSqDis;
                           minPointInd2 = j;
                        }
                     }
                     else
                     {
                        if (pointSqDis < minPointSqDis3)
                        {
                           minPointSqDis3 = pointSqDis;
                           minPointInd3 = j;
                        }
                     }
                  }
                  for (int j = closestPointInd - 1; j >= 0; j--)
                  {
                     if (int(_lastSurfaceCloud->points[j].intensity) < closestPointScan - 2.5)
                     {
                        break;
                     }

                     pointSqDis = calcSquaredDiff(_lastSurfaceCloud->points[j], pointSel);

                     if (int(_lastSurfaceCloud->points[j].intensity) >= closestPointScan)
                     {
                        if (pointSqDis < minPointSqDis2)
                        {
                           minPointSqDis2 = pointSqDis;
                           minPointInd2 = j;
                        }
                     }
                     else
                     {
                        if (pointSqDis < minPointSqDis3)
                        {
                           minPointSqDis3 = pointSqDis;
                           minPointInd3 = j;
                        }
                     }
                  }
               }

               _pointSearchSurfInd1[i] = closestPointInd;//kd-tree最近距离点,-1表示未找到满足要求的点
               _pointSearchSurfInd2[i] = minPointInd2;//同一线号上的距离最近的点，-1表示未找到满足要求的点
               _pointSearchSurfInd3[i] = minPointInd3;//不同线号上的距离最近的点，-1表示未找到满足要求的点
            }

            //计算planar point到上一帧点云中与之最近的planar的距离
            //参考论文"low-drift and real-time lidar odometry and mapping"公式3
            //点到平面的距离可用如下方法计算：
            //平面由三个点确定，这三个点可与平面外的一点构成四面体，定义平面三点构成向量b,c，平面外点与b、c交点构成向量a
            //因此体积可有这三个向量的点积和叉积确定：V=1/6*|a*(bxc)|
            //同时任意四面体的体积也可由棱锥的公式给出：V=1/3*A0*h，其中A0是平面三点构成的三角形的面积，h是平面外点到平面的距离
            //上述棱锥公式中的底面三角形面积A0可由向量叉积确定：A0=1/2*b×c
            //综合上述三式，点到平面的距离可表示为：h=|a*(b×c)|/|b×c|
            if (_pointSearchSurfInd2[i] >= 0 && _pointSearchSurfInd3[i] >= 0)
            {
               tripod1 = _lastSurfaceCloud->points[_pointSearchSurfInd1[i]];
               tripod2 = _lastSurfaceCloud->points[_pointSearchSurfInd2[i]];
               tripod3 = _lastSurfaceCloud->points[_pointSearchSurfInd3[i]];

               //向量(b×c)的三个分量
               //向量b=[t2.x-t1.x,t2.y-t1.y,t2.z-t1.z]，向量c=[t3.x-t1.x,t3.y-t1.y,t3.z-t1.z]
               //|    i          j          k    |
               //|t2.x-t1.x  t2.y-t1.y  t2.z-t1.z|
               //|t3.x-t1.x  t3.y-t1.y  t3.z-t1.z|
               float pa = (tripod2.y - tripod1.y) * (tripod3.z - tripod1.z)
                  - (tripod3.y - tripod1.y) * (tripod2.z - tripod1.z);
               float pb = (tripod2.z - tripod1.z) * (tripod3.x - tripod1.x)
                  - (tripod3.z - tripod1.z) * (tripod2.x - tripod1.x);
               float pc = (tripod2.x - tripod1.x) * (tripod3.y - tripod1.y)
                  - (tripod3.x - tripod1.x) * (tripod2.y - tripod1.y);
               /*
               //该部分将向量a拆分成两个点分别与向量(b×c)计算，难以理解
               float pd = -(pa * tripod1.x + pb * tripod1.y + pc * tripod1.z);

               float ps = sqrt(pa * pa + pb * pb + pc * pc);
               pa /= ps;
               pb /= ps;
               pc /= ps;
               pd /= ps;

               float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd; //Eq. (3)??
               */

               //重新按照公式编写过代码
               float ps = sqrt(pa * pa + pb * pb + pc * pc);//向量(b×c)的模
               float pd2 = ((pointSel.x-tripod1.x)*pa+(pointSel.y-tripod1.y)*pb+(pointSel.z-tripod1.z)*pc)/ps;//|a*(bxc)|/|(bxc)|
               //pa,pb,pc为向量(b×c)在坐标轴上的三个分量，而ps为该向量的模，因此这三者表示该向量方向的单位向量
               //同时注意到，向量(b×c)为其所在平面的法向量
               //因此前面的距离计算公式也可解释求为面外一点与面内一点所构成的向量在该单位向量方向上的投影
               //而实际上前面处理edge point时也是利用相同的原理
               //而之所以需要求出该单位向量的分量，为了方便求偏导，具体见下面的代码注解
               pa /= ps;
               pb /= ps;
               pc /= ps;

               // TODO: Why writing to a variable that's never read? Maybe it should be used afterwards?
               pointProj = pointSel;
               pointProj.x -= pa * pd2;
               pointProj.y -= pb * pd2;
               pointProj.z -= pc * pd2;

               //同理计算权重
               float s = 1;
               if (iterCount >= 5)
               {
                  s = 1 - 1.8f * fabs(pd2) / sqrt(calcPointDistance(pointSel));
               }

               coeff.x = s * pa;
               coeff.y = s * pb;
               coeff.z = s * pc;
               coeff.intensity = s * pd2;

               if (s > 0.1 && pd2 != 0)
               {
                  _laserCloudOri->push_back(_surfPointsFlat->points[i]);
                  _coeffSel->push_back(coeff);
               }
            }
         }

         int pointSelNum = _laserCloudOri->points.size();
         if (pointSelNum < 10)
         {//满足要求的特征点至少10个，特征匹配数量太少弃用此帧数据
            continue;
         }

         Eigen::Matrix<float, Eigen::Dynamic, 6> matA(pointSelNum, 6);//该矩阵每一行为偏导，多少列代表多少个点
         Eigen::Matrix<float, 6, Eigen::Dynamic> matAt(6, pointSelNum);//该矩阵等于矩阵matA的转置
         Eigen::Matrix<float, 6, 6> matAtA;//该矩阵等于矩阵A的转置乘以矩阵A
         Eigen::VectorXf matB(pointSelNum);//每一个点(不管是edge还是planar point)对应的距离
         Eigen::Matrix<float, 6, 1> matAtB;//该矩阵等于矩阵A的转置乘以矩阵B
         Eigen::Matrix<float, 6, 1> matX;//matAtA*matX=matAtB

         //当前点云中有多少个特征点(edge/planar point)就对应多少个方程
         for (int i = 0; i < pointSelNum; i++)
         {
            //此处是用_laserCloudOri中的点来建立方程，_laserCloudOri的点来自于当前点云帧中的特征点_cornerPointsSharp/_surfPointsFlat中可在上一帧点云中找到对应线/面的点，见248~560行代码
            //前面寻找其对应的上一帧点云中的线/面是提取_cornerPointsSharp/_surfPointsFlat中的点
            //赋给变量pointSel，pointSel再投影到当前点云帧的初始时刻(见261/414行代码)，进而寻找上一帧点云中与之对应的线/面
            //而在该部分，则是通过变换矩阵将特征点变换到初始时刻，这与通过poingSel投影到初始时刻起相同的作用
            //再利用高斯牛顿迭代法不断优化这个变换矩阵，最终可以得到当前帧从初始时刻到结束时刻，这个时间段的位姿变换
            //而前面pointSel投影到初始时刻则是基于匀速假设，投影所使用的位姿变换是上一帧点云的位姿变换
            const pcl::PointXYZI& pointOri = _laserCloudOri->points[i];
            coeff = _coeffSel->points[i];

            float s = 1;

            float srx = sin(s * _transform.rot_x.rad());
            float crx = cos(s * _transform.rot_x.rad());
            float sry = sin(s * _transform.rot_y.rad());
            float cry = cos(s * _transform.rot_y.rad());
            float srz = sin(s * _transform.rot_z.rad());
            float crz = cos(s * _transform.rot_z.rad());
            float tx = s * _transform.pos.x();
            float ty = s * _transform.pos.y();
            float tz = s * _transform.pos.z();

            //求偏导数
            //要求偏导数，首先要获得原函数，原函数将当前点云帧中的特征点(pointOri)通过位姿变换统一到上一帧点云的坐标系中，
            //然后求变换后的点到前面求得的直线/平面的距离，完整的论述见论文"low-drift and real-time lidar odometry and mapping"5.3节的公式6~10
            //如果该位姿变换是正确的，距离应该为0，不为0则通过高斯-牛顿法进行迭代使距离逼近0
            //X~=R*X+t，X表示当前点云帧特征点，R、t表示位姿变换，得到了X在上一帧点云坐标系下的表示X~
            //就可以求这一点到对应的线/平面的距离，此时前面求取的线/平面的单位法向量就派上用场了
            //f/T=dR/dΘ*[(pointOri.x-tx),(pointOri.x-tx),(pointOri.x-tx)]*[coeff.x,coeff.y,coeff.z]
            //旋转矩阵R由欧拉角表示，https://en.wikipedia.org/wiki/Euler_angles，R=Y1X2Z3
            float arx = (-s * crx*sry*srz*pointOri.x + s * crx*crz*sry*pointOri.y + s * srx*sry*pointOri.z
                         + s * tx*crx*sry*srz - s * ty*crx*crz*sry - s * tz*srx*sry) * coeff.x
               + (s*srx*srz*pointOri.x - s * crz*srx*pointOri.y + s * crx*pointOri.z
                  + s * ty*crz*srx - s * tz*crx - s * tx*srx*srz) * coeff.y
               + (s*crx*cry*srz*pointOri.x - s * crx*cry*crz*pointOri.y - s * cry*srx*pointOri.z
                  + s * tz*cry*srx + s * ty*crx*cry*crz - s * tx*crx*cry*srz) * coeff.z;

            float ary = ((-s * crz*sry - s * cry*srx*srz)*pointOri.x
                         + (s*cry*crz*srx - s * sry*srz)*pointOri.y - s * crx*cry*pointOri.z
                         + tx * (s*crz*sry + s * cry*srx*srz) + ty * (s*sry*srz - s * cry*crz*srx)
                         + s * tz*crx*cry) * coeff.x
               + ((s*cry*crz - s * srx*sry*srz)*pointOri.x
                  + (s*cry*srz + s * crz*srx*sry)*pointOri.y - s * crx*sry*pointOri.z
                  + s * tz*crx*sry - ty * (s*cry*srz + s * crz*srx*sry)
                  - tx * (s*cry*crz - s * srx*sry*srz)) * coeff.z;

            float arz = ((-s * cry*srz - s * crz*srx*sry)*pointOri.x + (s*cry*crz - s * srx*sry*srz)*pointOri.y
                         + tx * (s*cry*srz + s * crz*srx*sry) - ty * (s*cry*crz - s * srx*sry*srz)) * coeff.x
               + (-s * crx*crz*pointOri.x - s * crx*srz*pointOri.y
                  + s * ty*crx*srz + s * tx*crx*crz) * coeff.y
               + ((s*cry*crz*srx - s * sry*srz)*pointOri.x + (s*crz*sry + s * cry*srx*srz)*pointOri.y
                  + tx * (s*sry*srz - s * cry*crz*srx) - ty * (s*crz*sry + s * cry*srx*srz)) * coeff.z;

            float atx = -s * (cry*crz - srx * sry*srz) * coeff.x + s * crx*srz * coeff.y
               - s * (crz*sry + cry * srx*srz) * coeff.z;

            float aty = -s * (cry*srz + crz * srx*sry) * coeff.x - s * crx*crz * coeff.y
               - s * (sry*srz - cry * crz*srx) * coeff.z;

            float atz = s * crx*sry * coeff.x - s * srx * coeff.y - s * crx*cry * coeff.z;

            //float d2 = coeff.intensity;

            matA(i, 0) = arx;
            matA(i, 1) = ary;
            matA(i, 2) = arz;
            matA(i, 3) = atx;
            matA(i, 4) = aty;
            matA(i, 5) = atz;
            matB(i, 0) = -0.05 * coeff.intensity;
         }
         matAt = matA.transpose();//求转置
         matAtA = matAt * matA;//matAtA是矩阵A的转置乘以A
         matAtB = matAt * matB;//matAtB是矩阵A的转置乘以B

         //高斯牛顿法，根据迭代公式可得(matAt*matA)[x(k)-x(k+1)]=matAt*matB
         //其中x(k)表示上一次的位姿变换，matX=x(k)-x(k+1)，我们要求的是x(k+1)
         //求解线性方程组matAtA * matX = matAtB，参考http://eigen.tuxfamily.org/dox/classEigen_1_1ColPivHouseholderQR.html
         matX = matAtA.colPivHouseholderQr().solve(matAtB);

         //退化场景判断与处理
         if (iterCount == 0)
          {
            Eigen::Matrix<float, 1, 6> matE;//特征值1*6矩阵
            Eigen::Matrix<float, 6, 6> matV;//特征向量6*6矩阵
            Eigen::Matrix<float, 6, 6> matV2;

            //求解特征值/特征向量
            Eigen::SelfAdjointEigenSolver< Eigen::Matrix<float, 6, 6> > esolver(matAtA);
            matE = esolver.eigenvalues().real();
            matV = esolver.eigenvectors().real();

            matV2 = matV;

            isDegenerate = false;
            float eignThre[6] = { 10, 10, 10, 10, 10, 10 };//特征值取值门槛
            for (int i = 0; i < 6; i++)
            {//从小到大查找
               if (matE(0, i) < eignThre[i])
               {//特征值太小，则认为处在兼并环境中，发生了退化
                  for (int j = 0; j < 6; j++)
                  {
                     matV2(i, j) = 0;//对应的特征向量置为0
                  }
                  isDegenerate = true;
               }
               else
               {
                  break;
               }
            }
            matP = matV.inverse() * matV2;//计算P矩阵
         }

         if (isDegenerate)
         {//如果发生退化，只使用预测矩阵P计算
            Eigen::Matrix<float, 6, 1> matX2(matX);
            matX = matP * matX2;
         }

         //累加每次迭代的旋转平移量
         //按照前面求解matX的过程，理论上是x(k+1)=x(k)-matX
         //这里是加上matX，是因为给matB赋值的时候加了个负号-，见611行，因此matAt*matB整体多了个负号-
         _transform.rot_x = _transform.rot_x.rad() + matX(0, 0);
         _transform.rot_y = _transform.rot_y.rad() + matX(1, 0);
         _transform.rot_z = _transform.rot_z.rad() + matX(2, 0);
         _transform.pos.x() += matX(3, 0);
         _transform.pos.y() += matX(4, 0);
         _transform.pos.z() += matX(5, 0);

         //判断是否为有效数值
         if (!pcl_isfinite(_transform.rot_x.rad()))
            _transform.rot_x = Angle();
         if (!pcl_isfinite(_transform.rot_y.rad()))
            _transform.rot_y = Angle();
         if (!pcl_isfinite(_transform.rot_z.rad()))
            _transform.rot_z = Angle();
         if (!pcl_isfinite(_transform.pos.x()))
            _transform.pos.x() = 0.0;
         if (!pcl_isfinite(_transform.pos.y()))
            _transform.pos.y() = 0.0;
         if (!pcl_isfinite(_transform.pos.z()))
            _transform.pos.z() = 0.0;

         //计算旋转平移量
         float deltaR = sqrt(pow(rad2deg(matX(0, 0)), 2) +
                             pow(rad2deg(matX(1, 0)), 2) +
                             pow(rad2deg(matX(2, 0)), 2));
         float deltaT = sqrt(pow(matX(3, 0) * 100, 2) +
                             pow(matX(4, 0) * 100, 2) +
                             pow(matX(5, 0) * 100, 2));

         if (deltaR < _deltaRAbort && deltaT < _deltaTAbort)
            break;//如果很小就停止迭代
      }//迭代结束
   }

   Angle rx, ry, rz;
   //求相对于世界坐标系的旋转量,垂直方向上1.05倍修正?
   //注意，这里在_transform前面加上了负号，是因为迭代求得的是从当前帧结束时刻到初始时刻的欧拉角变换
   //现在需要求全局(世界)坐标下的当前帧结束时刻Lidar的欧拉角，则需要获得当前帧结束时刻相对于初始时刻的欧拉角变换
   //再和之前累积的进行运算
   accumulateRotation(_transformSum.rot_x,
                      _transformSum.rot_y,
                      _transformSum.rot_z,
                      -_transform.rot_x,
                      -_transform.rot_y.rad() * 1.05,
                      -_transform.rot_z,
                      rx, ry, rz);

   Vector3 v(_transform.pos.x() - _imuShiftFromStart.x(),
             _transform.pos.y() - _imuShiftFromStart.y(),
             _transform.pos.z() * 1.05 - _imuShiftFromStart.z());
   rotateZXY(v, rz, rx, ry);//将雷达坐标系下当前点云帧从初始时刻到结束时刻的位移变换到世界坐标系下
   //这里将当前帧从初始时刻到结束时刻的位移和之前所有时间的位移累加起来
   //之所以是减号，是因为在做高斯牛顿迭代优化的时候，是将当前帧的点都变换到初始时刻，再与上一帧求距离，优化迭代使距离逼近0
   //因此我们优化得到的变换是将点从当前帧的结束时刻变换到初始时刻
   Vector3 trans = _transformSum.pos - v;
/*
   //根据IMU修正旋转量
   pluginIMURotation(rx, ry, rz,
                     _imuPitchStart, _imuYawStart, _imuRollStart,
                     _imuPitchEnd, _imuYawEnd, _imuRollEnd,
                     rx, ry, rz);*/
   //得到当前点云帧结束时刻激光雷达在世界坐标系下的欧拉角和位置
   _transformSum.rot_x = rx;
   _transformSum.rot_y = ry;
   _transformSum.rot_z = rz;
   _transformSum.pos = trans;

   //对点云的曲率比较大和比较小的点投影到扫描结束位置
   transformToEnd(_cornerPointsLessSharp);
   transformToEnd(_surfPointsLessFlat);

   _cornerPointsLessSharp.swap(_lastCornerCloud);//
   _surfPointsLessFlat.swap(_lastSurfaceCloud);

   //畸变校正之后的点作为last点保存等下个点云进来进行匹配
   lastCornerCloudSize = _lastCornerCloud->points.size();
   lastSurfaceCloudSize = _lastSurfaceCloud->points.size();

   if (lastCornerCloudSize > 10 && lastSurfaceCloudSize > 100)
   {//点足够多就构建kd-tree，否则弃用此帧，沿用上一帧数据的kd-tree
      _lastCornerKDTree.setInputCloud(_lastCornerCloud);
      _lastSurfaceKDTree.setInputCloud(_lastSurfaceCloud);
   }
   
}



} // end namespace loam
