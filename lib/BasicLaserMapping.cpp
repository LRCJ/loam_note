// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.


#include "loam_velodyne/BasicLaserMapping.h"
#include "loam_velodyne/nanoflann_pcl.h"
#include "loam_velodyne/math_utils.h"

#include <Eigen/Eigenvalues>
#include <Eigen/QR>

namespace loam
{

using std::sqrt;
using std::fabs;
using std::asin;
using std::atan2;
using std::pow;


BasicLaserMapping::BasicLaserMapping():
   _scanPeriod(0.1),
   _stackFrameNum(1),
   _mapFrameNum(5),
   _frameCount(0),
   _mapFrameCount(0),
   _maxIterations(10),
   _deltaTAbort(0.05),
   _deltaRAbort(0.05),
   _laserCloudCenWidth(10),
   _laserCloudCenHeight(5),
   _laserCloudCenDepth(10),
   _laserCloudWidth(21),
   _laserCloudHeight(11),
   _laserCloudDepth(21),
   _laserCloudNum(_laserCloudWidth * _laserCloudHeight * _laserCloudDepth),//记录cube的总数，21*11*21=4851
   _laserCloudCornerLast(new pcl::PointCloud<pcl::PointXYZI>()),
   _laserCloudSurfLast(new pcl::PointCloud<pcl::PointXYZI>()),
   _laserCloudFullRes(new pcl::PointCloud<pcl::PointXYZI>()),
   _laserCloudCornerStack(new pcl::PointCloud<pcl::PointXYZI>()),
   _laserCloudSurfStack(new pcl::PointCloud<pcl::PointXYZI>()),
   _laserCloudCornerStackDS(new pcl::PointCloud<pcl::PointXYZI>()),
   _laserCloudSurfStackDS(new pcl::PointCloud<pcl::PointXYZI>()),
   _laserCloudSurround(new pcl::PointCloud<pcl::PointXYZI>()),
   _laserCloudSurroundDS(new pcl::PointCloud<pcl::PointXYZI>()),
   _laserCloudCornerFromMap(new pcl::PointCloud<pcl::PointXYZI>()),
   _laserCloudSurfFromMap(new pcl::PointCloud<pcl::PointXYZI>())
{
   // initialize frame counter
   _frameCount = _stackFrameNum - 1;//_frameCount = _stackFrameNum - 1 = 1 - 1 = 0
   _mapFrameCount = _mapFrameNum - 1;//_mapFrameCount = _mapFrameNum - 1 = 5 - 1 = 4

   // setup cloud vectors
   //地图，将整个特征点云划分为_laserCloudWidth * _laserCloudHeight * _laserCloudDepth=宽21*高11*长21=4851个cube
   //每一个cube为50m*50m*50m
   //这些cube是以一维向量的形式存储，每个元素代表了对应cube中的点云
   //定义为std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> _laserCloudCornerArray;
   _laserCloudCornerArray.resize(_laserCloudNum);
   _laserCloudSurfArray.resize(_laserCloudNum);
   _laserCloudCornerDSArray.resize(_laserCloudNum);//经过下采样后的点云
   _laserCloudSurfDSArray.resize(_laserCloudNum);

   for (size_t i = 0; i < _laserCloudNum; i++)
   {//每一个cube都存储相应位置的点云，特别需要注意的是，存储的是特征点云，而不是所有点云，
      _laserCloudCornerArray[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
      _laserCloudSurfArray[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
      _laserCloudCornerDSArray[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
      _laserCloudSurfDSArray[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
   }

   //设置下采样的网格大小
   _downSizeFilterCorner.setLeafSize(0.2, 0.2, 0.2);
   _downSizeFilterSurf.setLeafSize(0.4, 0.4, 0.4);
}

//基于匀速模型，根据上次微调的结果和odometry这次与上次计算的结果，猜测一个新的世界坐标系的转换矩阵transformTobeMapped
void BasicLaserMapping::transformAssociateToMap()
{
   _transformIncre.pos = _transformBefMapped.pos - _transformSum.pos;
   rotateYXZ(_transformIncre.pos, -(_transformSum.rot_y), -(_transformSum.rot_x), -(_transformSum.rot_z));

   //欧拉角--->旋转矩阵Rbc，这是由LaserOdometry节点得出的当前点云帧结束时刻Lidar相对世界坐标系的姿态欧拉角
   //| cbcy*cbcz+sbcx*sbcy*sbcz , sbcx*sbcy*cbcz-cbcy*sbcz , cbcx*sbcy |
   //|      cbcx*sbcz           ,         cbcx*cbcz        ,   -sbcx   |
   //| sbcx*cbcy*sbcz-sbcy*cbcz , sbcx*cbcy*cbcz+sbcy*sbcz , cbcy*cbcx |
   float sbcx = _transformSum.rot_x.sin();
   float cbcx = _transformSum.rot_x.cos();
   float sbcy = _transformSum.rot_y.sin();
   float cbcy = _transformSum.rot_y.cos();
   float sbcz = _transformSum.rot_z.sin();
   float cbcz = _transformSum.rot_z.cos();


   //欧拉角--->旋转矩阵Rbl
   //| cbly*cblz+sblx*sbly*sblz , sblx*sbly*cblz-cbly*sblz , cblx*sbly |
   //|      cblx*sblz           ,         cblx*cblz        ,   -sblx   |
   //| sblx*cbly*sblz-sbly*cblz , sblx*cbly*cblz+sbly*sblz , cbly*cblx |
   //Rbl转置
   //| cbly*cblz+sblx*sbly*sblz , cblx*sblz , sblx*cbly*sblz-sbly*cblz |
   //| sblx*sbly*cblz-cbly*sblz , cblx*cblz , sblx*cbly*cblz+sbly*sblz |
   //|        cblx*sbly         ,   -sblx   ,        cbly*cblx         |
   float sblx = _transformBefMapped.rot_x.sin();
   float cblx = _transformBefMapped.rot_x.cos();
   float sbly = _transformBefMapped.rot_y.sin();
   float cbly = _transformBefMapped.rot_y.cos();
   float sblz = _transformBefMapped.rot_z.sin();
   float cblz = _transformBefMapped.rot_z.cos();


   //欧拉角--->旋转矩阵Ral
   //| caly*calz+salx*saly*salz , salx*saly*calz-caly*salz , calx*saly |
   //|      calx*salz           ,         calx*calz        ,   -salx   |
   //| salx*caly*salz-saly*calz , salx*caly*calz+saly*salz , caly*calx |
   //Ral转置
   //| caly*calz+salx*saly*salz , calx*salz , salx*caly*salz-saly*calz |
   //| salx*saly*calz-caly*salz , calx*calz , salx*caly*calz+saly*salz |
   //|        calx*saly         ,   -salx   ,        caly*calx         |
   float salx = _transformAftMapped.rot_x.sin();
   float calx = _transformAftMapped.rot_x.cos();
   float saly = _transformAftMapped.rot_y.sin();
   float caly = _transformAftMapped.rot_y.cos();
   float salz = _transformAftMapped.rot_z.sin();
   float calz = _transformAftMapped.rot_z.cos();

   //R = Ral*Rbl转置*Rbc，此处的Ral和Rbl，或者说_transformBefMapped和_transformAftMapped都是在处理上一帧点云中得到的
   //Ral*Rbl转置等于上一帧Mapping算法的优化量，此处直接乘上Rbc即是假设当前帧的优化量等于上一帧的优化量，作为一个初始值
   float srx = -sbcx * (salx*sblx + calx * cblx*salz*sblz + calx * calz*cblx*cblz)
      - cbcx * sbcy*(calx*calz*(cbly*sblz - cblz * sblx*sbly)
                     - calx * salz*(cbly*cblz + sblx * sbly*sblz) + cblx * salx*sbly)
      - cbcx * cbcy*(calx*salz*(cblz*sbly - cbly * sblx*sblz)
                     - calx * calz*(sbly*sblz + cbly * cblz*sblx) + cblx * cbly*salx);
   _transformTobeMapped.rot_x = -asin(srx);

   float srycrx = sbcx * (cblx*cblz*(caly*salz - calz * salx*saly)
                          - cblx * sblz*(caly*calz + salx * saly*salz) + calx * saly*sblx)
      - cbcx * cbcy*((caly*calz + salx * saly*salz)*(cblz*sbly - cbly * sblx*sblz)
                     + (caly*salz - calz * salx*saly)*(sbly*sblz + cbly * cblz*sblx) - calx * cblx*cbly*saly)
      + cbcx * sbcy*((caly*calz + salx * saly*salz)*(cbly*cblz + sblx * sbly*sblz)
                     + (caly*salz - calz * salx*saly)*(cbly*sblz - cblz * sblx*sbly) + calx * cblx*saly*sbly);
   float crycrx = sbcx * (cblx*sblz*(calz*saly - caly * salx*salz)
                          - cblx * cblz*(saly*salz + caly * calz*salx) + calx * caly*sblx)
      + cbcx * cbcy*((saly*salz + caly * calz*salx)*(sbly*sblz + cbly * cblz*sblx)
                     + (calz*saly - caly * salx*salz)*(cblz*sbly - cbly * sblx*sblz) + calx * caly*cblx*cbly)
      - cbcx * sbcy*((saly*salz + caly * calz*salx)*(cbly*sblz - cblz * sblx*sbly)
                     + (calz*saly - caly * salx*salz)*(cbly*cblz + sblx * sbly*sblz) - calx * caly*cblx*sbly);
   _transformTobeMapped.rot_y = atan2(srycrx / _transformTobeMapped.rot_x.cos(),
                                      crycrx / _transformTobeMapped.rot_x.cos());

   float srzcrx = (cbcz*sbcy - cbcy * sbcx*sbcz)*(calx*salz*(cblz*sbly - cbly * sblx*sblz)
                                                  - calx * calz*(sbly*sblz + cbly * cblz*sblx) + cblx * cbly*salx)
      - (cbcy*cbcz + sbcx * sbcy*sbcz)*(calx*calz*(cbly*sblz - cblz * sblx*sbly)
                                        - calx * salz*(cbly*cblz + sblx * sbly*sblz) + cblx * salx*sbly)
      + cbcx * sbcz*(salx*sblx + calx * cblx*salz*sblz + calx * calz*cblx*cblz);
   float crzcrx = (cbcy*sbcz - cbcz * sbcx*sbcy)*(calx*calz*(cbly*sblz - cblz * sblx*sbly)
                                                  - calx * salz*(cbly*cblz + sblx * sbly*sblz) + cblx * salx*sbly)
      - (sbcy*sbcz + cbcy * cbcz*sbcx)*(calx*salz*(cblz*sbly - cbly * sblx*sblz)
                                        - calx * calz*(sbly*sblz + cbly * cblz*sblx) + cblx * cbly*salx)
      + cbcx * cbcz*(salx*sblx + calx * cblx*salz*sblz + calx * calz*cblx*cblz);
   _transformTobeMapped.rot_z = atan2(srzcrx / _transformTobeMapped.rot_x.cos(),
                                      crzcrx / _transformTobeMapped.rot_x.cos());

   Vector3 v = _transformIncre.pos;
   rotateZXY(v, _transformTobeMapped.rot_z, _transformTobeMapped.rot_x, _transformTobeMapped.rot_y);
   _transformTobeMapped.pos = _transformAftMapped.pos - v;
}


//位姿优化结束后执行该函数
//记录odometry发送的转换矩阵与mapping之后的转换矩阵，下一帧点云会使用(有IMU的话会使用IMU进行补偿)
void BasicLaserMapping::transformUpdate()
{/*
   if (0 < _imuHistory.size())
   {
      size_t imuIdx = 0;

      while (imuIdx < _imuHistory.size() - 1 && toSec(_laserOdometryTime - _imuHistory[imuIdx].stamp) + _scanPeriod > 0)
      {
         imuIdx++;
      }

      IMUState2 imuCur;

      if (imuIdx == 0 || toSec(_laserOdometryTime - _imuHistory[imuIdx].stamp) + _scanPeriod > 0)
      {
         // scan time newer then newest or older than oldest IMU message
         imuCur = _imuHistory[imuIdx];
      }
      else
      {
         float ratio = (toSec(_imuHistory[imuIdx].stamp - _laserOdometryTime) - _scanPeriod)
            / toSec(_imuHistory[imuIdx].stamp - _imuHistory[imuIdx - 1].stamp);

         IMUState2::interpolate(_imuHistory[imuIdx], _imuHistory[imuIdx - 1], ratio, imuCur);
      }

      _transformTobeMapped.rot_x = 0.998 * _transformTobeMapped.rot_x.rad() + 0.002 * imuCur.pitch.rad();
      _transformTobeMapped.rot_z = 0.998 * _transformTobeMapped.rot_z.rad() + 0.002 * imuCur.roll.rad();
   }*/

   _transformBefMapped = _transformSum;//未优化之前的，由odometry输出的位姿
   _transformAftMapped = _transformTobeMapped;//最终优化的位姿
}


//根据优化计算后的位姿变换，将点转换到全局世界坐标系下
void BasicLaserMapping::pointAssociateToMap(const pcl::PointXYZI& pi, pcl::PointXYZI& po)
{
   po.x = pi.x;
   po.y = pi.y;
   po.z = pi.z;
   po.intensity = pi.intensity;

   rotateZXY(po, _transformTobeMapped.rot_z, _transformTobeMapped.rot_x, _transformTobeMapped.rot_y);

   po.x += _transformTobeMapped.pos.x();
   po.y += _transformTobeMapped.pos.y();
   po.z += _transformTobeMapped.pos.z();
}


//点转移到局部坐标系下
void BasicLaserMapping::pointAssociateTobeMapped(const pcl::PointXYZI& pi, pcl::PointXYZI& po)
{
   po.x = pi.x - _transformTobeMapped.pos.x();
   po.y = pi.y - _transformTobeMapped.pos.y();
   po.z = pi.z - _transformTobeMapped.pos.z();
   po.intensity = pi.intensity;

   rotateYXZ(po, -_transformTobeMapped.rot_y, -_transformTobeMapped.rot_x, -_transformTobeMapped.rot_z);
}


//将当前点云帧的所有点转换到全局坐标系下
void BasicLaserMapping::transformFullResToMap()
{
   // transform full resolution input cloud to map
   for (auto& pt : *_laserCloudFullRes)
      pointAssociateToMap(pt, pt);
}

//
bool BasicLaserMapping::createDownsizedMap()
{
   // create new map cloud according to the input output ratio
   //首次进入该函数时_mapFrameCount = 4，_mapFrameNum = 5，然后下面的if不执行，即不返回false并且_mapFrameCount被置为0
   //此后，4次进入该函数都执行if，直接返回false，导致LaserMapping::publishResult()函数不publish _laserCloudSurroundDS
   //数据的topic，只publish优化过的位姿和当前帧所有的点云
   _mapFrameCount++;
   if (_mapFrameCount < _mapFrameNum)
      return false;

   _mapFrameCount = 0;

   // accumulate map cloud
   _laserCloudSurround->clear();
   for (auto ind : _laserCloudSurroundInd)
   {//将当前Lidar位于的cube的周围125个(5*5*5)cube且处于Lidar可视范围内的cube中的点云集中存储到_laserCloudSurround
      *_laserCloudSurround += *_laserCloudCornerArray[ind];
      *_laserCloudSurround += *_laserCloudSurfArray[ind];
   }

   //对_laserCloudSurround进行下采样，存储至_laserCloudSurroundDS
   _laserCloudSurroundDS->clear();
   _downSizeFilterCorner.setInputCloud(_laserCloudSurround);
   _downSizeFilterCorner.filter(*_laserCloudSurroundDS);
   return true;
}

bool BasicLaserMapping::process(Time const& laserOdometryTime,char* log)
{
   char s[50];//log info
   // skip some frames???
   _frameCount++;// _frameCount = 0
   if (_frameCount < _stackFrameNum) // _stackFrameNum = 1
   {
      sprintf(s,"_frameCount(=%ld)<_stackFrameNum(=%d)",_frameCount,_stackFrameNum);
      strcpy(log,s);
      return false;
   }
   _frameCount = 0;
   _laserOdometryTime = laserOdometryTime;//接收来自LaserOdometry节点的位姿信息的时间戳

   pcl::PointXYZI pointSel;

   // 根据上一次mapping的位姿优化结果对此次mapping赋予一个初始位姿
   transformAssociateToMap();

   //将当前帧的edge point和planar point转换到世界坐标系下
   for (auto const& pt : _laserCloudCornerLast->points)
   {
      pointAssociateToMap(pt, pointSel);
      _laserCloudCornerStack->push_back(pointSel);
   }

   for (auto const& pt : _laserCloudSurfLast->points)
   {
      pointAssociateToMap(pt, pointSel);
      _laserCloudSurfStack->push_back(pointSel);
   }

   //获取y方向上10米高位置的点在世界坐标系下的坐标，该点只在后面判断cube是否在Lidar可视范围内使用了
   pcl::PointXYZI pointOnYAxis;
   pointOnYAxis.x = 0.0;
   pointOnYAxis.y = 10.0;
   pointOnYAxis.z = 0.0;
   pointAssociateToMap(pointOnYAxis, pointOnYAxis);

   //每一个cube大小为50m*50m*50m
   auto const CUBE_SIZE = 50.0;
   auto const CUBE_HALF = CUBE_SIZE / 2;

   //_laserCloudCenWidth=10,_laserCloudCenHeight=5,_laserCloudCenDepth=10
   //地图被划分为了宽21*高11*长21=4851个cube，centerCubeI/J/K分别表示当前点云帧的结束时刻Lidar在地图中的IJK(对应宽高长)坐标，
   //加上偏移量是保证IJK坐标不为负数，因为cube存储在_laserCloudCornerArray/_laserCloudSurfArray数组中，而数组下标不能为负的
   //偏移laserCloudCenWidth/Heigh/Depth会动态调整，来保证当前位置尽量位于地图的中心处
   int centerCubeI = int((_transformTobeMapped.pos.x() + CUBE_HALF) / CUBE_SIZE) + _laserCloudCenWidth;
   int centerCubeJ = int((_transformTobeMapped.pos.y() + CUBE_HALF) / CUBE_SIZE) + _laserCloudCenHeight;
   int centerCubeK = int((_transformTobeMapped.pos.z() + CUBE_HALF) / CUBE_SIZE) + _laserCloudCenDepth;

   //由于计算机求余是向零取整，为了不使（-50.0,50.0）求余后都向零偏移，当被求余数为负数时求余结果统一向左偏移一个单位，也即减一
   if (_transformTobeMapped.pos.x() + CUBE_HALF < 0) centerCubeI--;
   if (_transformTobeMapped.pos.y() + CUBE_HALF < 0) centerCubeJ--;
   if (_transformTobeMapped.pos.z() + CUBE_HALF < 0) centerCubeK--;

   //IJK坐标取值范围:3 < centerCubeI < 18， 3 < centerCubeJ < 8, 3 < centerCubeK < 18
   //当centerCubeI/J/K超出预定的边界时，将地图整体移动，使centerCubeI/J/K回到整个地图的中心
   //
   //如果处于下边界，表明地图向负方向延伸的可能性比较大，则循环移位，将数组中心点向上边界调整一个单位
   while (centerCubeI < 3)
   {
      for (int j = 0; j < _laserCloudHeight; j++)//_laserCloudHeight = 11
      {
         for (int k = 0; k < _laserCloudDepth; k++)//_laserCloudDepth = 21
         {
            for (int i = _laserCloudWidth - 1; i >= 1; i--)//_laserCloudWidth = 21
            {
               const size_t indexA = toIndex(i, j, k);
               const size_t indexB = toIndex(i - 1, j, k);
               std::swap(_laserCloudCornerArray[indexA], _laserCloudCornerArray[indexB]);
               std::swap(_laserCloudSurfArray[indexA], _laserCloudSurfArray[indexB]);
            }
            const size_t indexC = toIndex(0, j, k);
            _laserCloudCornerArray[indexC]->clear();
            _laserCloudSurfArray[indexC]->clear();
         }
      }
      centerCubeI++;
      _laserCloudCenWidth++;
   }

   //如果处于上边界，表明地图向正方向延伸的可能性比较大，则循环移位，将数组中心点向下边界调整一个单位
   while (centerCubeI >= _laserCloudWidth - 3)//_laserCloudWidth=21，即在Width方向上即将达到上边界
   {
      for (int j = 0; j < _laserCloudHeight; j++)
      {
         for (int k = 0; k < _laserCloudDepth; k++)
         {
            for (int i = 0; i < _laserCloudWidth - 1; i++)
            {
               const size_t indexA = toIndex(i, j, k);
               const size_t indexB = toIndex(i + 1, j, k);
               std::swap(_laserCloudCornerArray[indexA], _laserCloudCornerArray[indexB]);
               std::swap(_laserCloudSurfArray[indexA], _laserCloudSurfArray[indexB]);
            }
            const size_t indexC = toIndex(_laserCloudWidth - 1, j, k);
            _laserCloudCornerArray[indexC]->clear();
            _laserCloudSurfArray[indexC]->clear();
         }
      }
      centerCubeI--;
      _laserCloudCenWidth--;
   }

   while (centerCubeJ < 3)
   {
      for (int i = 0; i < _laserCloudWidth; i++)
      {
         for (int k = 0; k < _laserCloudDepth; k++)
         {
            for (int j = _laserCloudHeight - 1; j >= 1; j--)
            {
               const size_t indexA = toIndex(i, j, k);
               const size_t indexB = toIndex(i, j - 1, k);
               std::swap(_laserCloudCornerArray[indexA], _laserCloudCornerArray[indexB]);
               std::swap(_laserCloudSurfArray[indexA], _laserCloudSurfArray[indexB]);
            }
            const size_t indexC = toIndex(i, 0, k);
            _laserCloudCornerArray[indexC]->clear();
            _laserCloudSurfArray[indexC]->clear();
         }
      }
      centerCubeJ++;
      _laserCloudCenHeight++;
   }

   while (centerCubeJ >= _laserCloudHeight - 3)
   {
      for (int i = 0; i < _laserCloudWidth; i++)
      {
         for (int k = 0; k < _laserCloudDepth; k++)
         {
            for (int j = 0; j < _laserCloudHeight - 1; j++)
            {
               const size_t indexA = toIndex(i, j, k);
               const size_t indexB = toIndex(i, j + 1, k);
               std::swap(_laserCloudCornerArray[indexA], _laserCloudCornerArray[indexB]);
               std::swap(_laserCloudSurfArray[indexA], _laserCloudSurfArray[indexB]);
            }
            const size_t indexC = toIndex(i, _laserCloudHeight - 1, k);
            _laserCloudCornerArray[indexC]->clear();
            _laserCloudSurfArray[indexC]->clear();
         }
      }
      centerCubeJ--;
      _laserCloudCenHeight--;
   }

   while (centerCubeK < 3)
   {
      for (int i = 0; i < _laserCloudWidth; i++)
      {
         for (int j = 0; j < _laserCloudHeight; j++)
         {
            for (int k = _laserCloudDepth - 1; k >= 1; k--)
            {
               const size_t indexA = toIndex(i, j, k);
               const size_t indexB = toIndex(i, j, k - 1);
               std::swap(_laserCloudCornerArray[indexA], _laserCloudCornerArray[indexB]);
               std::swap(_laserCloudSurfArray[indexA], _laserCloudSurfArray[indexB]);
            }
            const size_t indexC = toIndex(i, j, 0);
            _laserCloudCornerArray[indexC]->clear();
            _laserCloudSurfArray[indexC]->clear();
         }
      }
      centerCubeK++;
      _laserCloudCenDepth++;
   }

   while (centerCubeK >= _laserCloudDepth - 3)
   {
      for (int i = 0; i < _laserCloudWidth; i++)
      {
         for (int j = 0; j < _laserCloudHeight; j++)
         {
            for (int k = 0; k < _laserCloudDepth - 1; k++)
            {
               const size_t indexA = toIndex(i, j, k);
               const size_t indexB = toIndex(i, j, k + 1);
               std::swap(_laserCloudCornerArray[indexA], _laserCloudCornerArray[indexB]);
               std::swap(_laserCloudSurfArray[indexA], _laserCloudSurfArray[indexB]);
            }
            const size_t indexC = toIndex(i, j, _laserCloudDepth - 1);
            _laserCloudCornerArray[indexC]->clear();
            _laserCloudSurfArray[indexC]->clear();
         }
      }
      centerCubeK--;
      _laserCloudCenDepth--;
   }

   _laserCloudValidInd.clear();//存储以当前Lidar所在的cube为中心的125个处于Lidar可视范围内的cube的索引
   _laserCloudSurroundInd.clear();//存储以当前Lidar所在的cube为中心的125个cube索引
   //向IJK正负方向各扩展2个cube，IJK方向各5个cube，总共125个cube，用于和当前点云帧进行特征匹配
   for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++)
   {
      for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++)
      {
         for (int k = centerCubeK - 2; k <= centerCubeK + 2; k++)
         {
            if (i >= 0 && i < _laserCloudWidth &&
                j >= 0 && j < _laserCloudHeight &&
                k >= 0 && k < _laserCloudDepth)
            {
               //拓展cube在世界坐标系下的位姿
               float centerX = 50.0f * (i - _laserCloudCenWidth);
               float centerY = 50.0f * (j - _laserCloudCenHeight);
               float centerZ = 50.0f * (k - _laserCloudCenDepth);

               pcl::PointXYZI transform_pos = (pcl::PointXYZI) _transformTobeMapped.pos;

               bool isInLaserFOV = false;//判断是否在lidar视线范围内的标志
               for (int ii = -1; ii <= 1; ii += 2)
               {
                  for (int jj = -1; jj <= 1; jj += 2)
                  {
                     for (int kk = -1; kk <= 1; kk += 2)
                     {
                        //cube的8个顶点
                        pcl::PointXYZI corner;
                        corner.x = centerX + 25.0f * ii;
                        corner.y = centerY + 25.0f * jj;
                        corner.z = centerZ + 25.0f * kk;

                        //当前点云帧结束时刻lidar位置到cube顶点距离的平方
                        float squaredSide1 = calcSquaredDiff(transform_pos, corner);
                        //pointOnYAxis到顶点距离的平方
                        float squaredSide2 = calcSquaredDiff(pointOnYAxis, corner);

                        //利用了余弦公式判断cube是否在lidar的可视范围，可视范围取垂直视场角正负60°
                        //               ^
                        //               |    /         *---*
                        //               |   /          |   | 
                        //               *  /           *---*
                        //               | /
                        //---------------*/------------------>
                        //               |\
                        //               | \
                        //               |  \
                        //               |   \
                        //               |    \
                        //利用当前点云帧坐标原点，pointOnYAxis，以及一个cube的任意一个顶点构成三角形
                        //pointOnYAxis到原点距离等于10，cube顶点到原点距离平方等于squaredSide1，两者构成余弦公式中的两邻边a、b
                        //pointOnYAxis到cube顶点距离平方等于squaredSide2，作为余弦公式中的对边c
                        //根据cube应完全落在视场角内的要求，两邻边ab的角度应大于30°或小于150°才能保证cube落在lidar的可视范围内
                        //(a^2+b^2-c^2)/2*a*b<cos30°
                        float check1 = 100.0f + squaredSide1 - squaredSide2
                           - 10.0f * sqrt(3.0f) * sqrt(squaredSide1);//大于30°
                        //(a^2+b^2-c^2)/2*a*b>cos150°
                        float check2 = 100.0f + squaredSide1 - squaredSide2
                           + 10.0f * sqrt(3.0f) * sqrt(squaredSide1);//小于150°

                        if (check1 < 0 && check2 > 0)
                        {
                           isInLaserFOV = true;
                        }
                     }
                  }
               }
               //记录下125个cube中处于可视范围内的cube的索引
               size_t cubeIdx = i + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * k;
               if (isInLaserFOV)
               {
                  _laserCloudValidInd.push_back(cubeIdx);
               }
               //记录下125个cube的索引
               _laserCloudSurroundInd.push_back(cubeIdx);
            }
         }
      }
   }

   // prepare valid map corner and surface cloud for pose optimization
   _laserCloudCornerFromMap->clear();//
   _laserCloudSurfFromMap->clear();
   for (auto const& ind : _laserCloudValidInd)
   {
      *_laserCloudCornerFromMap += *_laserCloudCornerArray[ind];
      *_laserCloudSurfFromMap += *_laserCloudSurfArray[ind];
   }

   // prepare feature stack clouds for pose optimization
   for (auto& pt : *_laserCloudCornerStack)//在process()函数的开始，312行左右
      pointAssociateTobeMapped(pt, pt);//将特征点变换回当前点云帧结束时刻的lidar坐标系下

   for (auto& pt : *_laserCloudSurfStack)
      pointAssociateTobeMapped(pt, pt);

   //下采样
   // down sample feature stack clouds
   _laserCloudCornerStackDS->clear();
   _downSizeFilterCorner.setInputCloud(_laserCloudCornerStack);
   _downSizeFilterCorner.filter(*_laserCloudCornerStackDS);
   size_t laserCloudCornerStackNum = _laserCloudCornerStackDS->size();

   _laserCloudSurfStackDS->clear();
   _downSizeFilterSurf.setInputCloud(_laserCloudSurfStack);
   _downSizeFilterSurf.filter(*_laserCloudSurfStackDS);
   size_t laserCloudSurfStackNum = _laserCloudSurfStackDS->size();

   _laserCloudCornerStack->clear();
   _laserCloudSurfStack->clear();

   // run pose optimization
   optimizeTransformTobeMapped(s);
   strcpy(log,s);

   // store down sized corner stack points in corresponding cube clouds
   //将edge point归入对应的cube中
   for (int i = 0; i < laserCloudCornerStackNum; i++)
   {
      pointAssociateToMap(_laserCloudCornerStackDS->points[i], pointSel);

      int cubeI = int((pointSel.x + CUBE_HALF) / CUBE_SIZE) + _laserCloudCenWidth;
      int cubeJ = int((pointSel.y + CUBE_HALF) / CUBE_SIZE) + _laserCloudCenHeight;
      int cubeK = int((pointSel.z + CUBE_HALF) / CUBE_SIZE) + _laserCloudCenDepth;

      if (pointSel.x + CUBE_HALF < 0) cubeI--;
      if (pointSel.y + CUBE_HALF < 0) cubeJ--;
      if (pointSel.z + CUBE_HALF < 0) cubeK--;

      if (cubeI >= 0 && cubeI < _laserCloudWidth &&
          cubeJ >= 0 && cubeJ < _laserCloudHeight &&
          cubeK >= 0 && cubeK < _laserCloudDepth)
      {
         size_t cubeInd = cubeI + _laserCloudWidth * cubeJ + _laserCloudWidth * _laserCloudHeight * cubeK;
         _laserCloudCornerArray[cubeInd]->push_back(pointSel);
      }
   }

   // store down sized surface stack points in corresponding cube clouds
   //将planar point归入对应的cube中
   for (int i = 0; i < laserCloudSurfStackNum; i++)
   {
      pointAssociateToMap(_laserCloudSurfStackDS->points[i], pointSel);

      int cubeI = int((pointSel.x + CUBE_HALF) / CUBE_SIZE) + _laserCloudCenWidth;
      int cubeJ = int((pointSel.y + CUBE_HALF) / CUBE_SIZE) + _laserCloudCenHeight;
      int cubeK = int((pointSel.z + CUBE_HALF) / CUBE_SIZE) + _laserCloudCenDepth;

      if (pointSel.x + CUBE_HALF < 0) cubeI--;
      if (pointSel.y + CUBE_HALF < 0) cubeJ--;
      if (pointSel.z + CUBE_HALF < 0) cubeK--;

      if (cubeI >= 0 && cubeI < _laserCloudWidth &&
          cubeJ >= 0 && cubeJ < _laserCloudHeight &&
          cubeK >= 0 && cubeK < _laserCloudDepth)
      {
         size_t cubeInd = cubeI + _laserCloudWidth * cubeJ + _laserCloudWidth * _laserCloudHeight * cubeK;
         _laserCloudSurfArray[cubeInd]->push_back(pointSel);
      }
   }

   // down size all valid (within field of view) feature cube clouds
   // 对
   for (auto const& ind : _laserCloudValidInd)
   {
      _laserCloudCornerDSArray[ind]->clear();
      _downSizeFilterCorner.setInputCloud(_laserCloudCornerArray[ind]);
      _downSizeFilterCorner.filter(*_laserCloudCornerDSArray[ind]);

      _laserCloudSurfDSArray[ind]->clear();
      _downSizeFilterSurf.setInputCloud(_laserCloudSurfArray[ind]);
      _downSizeFilterSurf.filter(*_laserCloudSurfDSArray[ind]);

      // swap cube clouds for next processing
      _laserCloudCornerArray[ind].swap(_laserCloudCornerDSArray[ind]);
      _laserCloudSurfArray[ind].swap(_laserCloudSurfDSArray[ind]);
   }

   transformFullResToMap();//将所有点转换到世界坐标系下
   _downsizedMapCreated = createDownsizedMap();//所有处理步骤完成，将_downsizedMapCreated置为true，可publish相关数据的topic

   return true;
}


void BasicLaserMapping::updateIMU(IMUState2 const& newState)
{
   _imuHistory.push(newState);
}

void BasicLaserMapping::updateOdometry(double pitch, double yaw, double roll, double x, double y, double z)
{
   _transformSum.rot_x = pitch;
   _transformSum.rot_y = yaw;
   _transformSum.rot_z = roll;

   _transformSum.pos.x() = float(x);
   _transformSum.pos.y() = float(y);
   _transformSum.pos.z() = float(z);
}

void BasicLaserMapping::updateOdometry(Twist const& twist)
{
   _transformSum = twist;
}

nanoflann::KdTreeFLANN<pcl::PointXYZI> kdtreeCornerFromMap;
nanoflann::KdTreeFLANN<pcl::PointXYZI> kdtreeSurfFromMap;

//优化位姿
void BasicLaserMapping::optimizeTransformTobeMapped(char* log)
{
   char s[50];
   if (_laserCloudCornerFromMap->size() <= 10 || _laserCloudSurfFromMap->size() <= 100)
   {
      sprintf(s,"There are few feature points!Stop optimization!");
      strcpy(log,s);
      return;
   }

   pcl::PointXYZI pointSel, pointOri, /*pointProj, */coeff;

   std::vector<int> pointSearchInd(5, 0);
   std::vector<float> pointSearchSqDis(5, 0);

   kdtreeCornerFromMap.setInputCloud(_laserCloudCornerFromMap);
   kdtreeSurfFromMap.setInputCloud(_laserCloudSurfFromMap);

   Eigen::Matrix<float, 5, 3> matA0;
   Eigen::Matrix<float, 5, 1> matB0;
   Eigen::Vector3f matX0;
   Eigen::Matrix3f matA1;
   Eigen::Matrix<float, 1, 3> matD1;
   Eigen::Matrix3f matV1;

   matA0.setZero();
   matB0.setConstant(-1);
   matX0.setZero();

   matA1.setZero();
   matD1.setZero();
   matV1.setZero();

   bool isDegenerate = false;
   Eigen::Matrix<float, 6, 6> matP;

   size_t laserCloudCornerStackNum = _laserCloudCornerStackDS->size();
   size_t laserCloudSurfStackNum = _laserCloudSurfStackDS->size();
   size_t iterCount;
   for (iterCount = 0; iterCount < _maxIterations; iterCount++)//最大迭代次数10次，_maxIterations=10
   {
      _laserCloudOri.clear();
      _coeffSel.clear();

      //处理edge point
      for (int i = 0; i < laserCloudCornerStackNum; i++)
      {
         pointOri = _laserCloudCornerStackDS->points[i];
         pointAssociateToMap(pointOri, pointSel);//将当前点云帧中的特征点转换回世界坐标系
         kdtreeCornerFromMap.nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);//寻找最近的5个点

         if (pointSearchSqDis[4] < 1.0)//最大距离不超过1才处理
         {
            Vector3 vc(0, 0, 0);

            for (int j = 0; j < 5; j++)
               vc += Vector3(_laserCloudCornerFromMap->points[pointSearchInd[j]]);
            vc /= 5.0;//5个点的xyz坐标求平均值，可理解为质心

            Eigen::Matrix3f mat_a;
            mat_a.setZero();

            //求协方差
            for (int j = 0; j < 5; j++)
            {
               Vector3 a = Vector3(_laserCloudCornerFromMap->points[pointSearchInd[j]]) - vc;

               mat_a(0, 0) += a.x() * a.x();
               mat_a(1, 0) += a.x() * a.y();
               mat_a(2, 0) += a.x() * a.z();
               mat_a(1, 1) += a.y() * a.y();
               mat_a(2, 1) += a.y() * a.z();
               mat_a(2, 2) += a.z() * a.z();
            }
            matA1 = mat_a / 5.0;
            // This solver only looks at the lower-triangular part of matA1.
            //此处求当前点云特征点对应线/面的理论参考论文"Low-drift and real-time lidar odometry and mapping"第6节 Lidar mapping
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> esolver(matA1);
            matD1 = esolver.eigenvalues().real();//特征值Eigen::Matrix<float, 1, 3> matD1;
            matV1 = esolver.eigenvectors().real();//特征向量Eigen::Matrix3f matV1;

            if (matD1(0, 2) > 3 * matD1(0, 1))//如果最大的特征值大于第二大的特征值三倍以上，特征值默认从小到大排列
            {
               //(x0,y0,z0)代表特征点
               float x0 = pointSel.x;
               float y0 = pointSel.y;
               float z0 = pointSel.z;
               //(x1,y1,z1)和(x2,y2,z2)代表edge line上的两点
               float x1 = vc.x() + 0.1 * matV1(0, 2);
               float y1 = vc.y() + 0.1 * matV1(1, 2);
               float z1 = vc.z() + 0.1 * matV1(2, 2);
               float x2 = vc.x() - 0.1 * matV1(0, 2);
               float y2 = vc.y() - 0.1 * matV1(1, 2);
               float z2 = vc.z() - 0.1 * matV1(2, 2);
               //与LaserOdometry节点求点到直线的距离方法相同
               float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                                 * ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                                 + ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                                 * ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                                 + ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))
                                 * ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));

               float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

               float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                           + (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

               float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                            - (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

               float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                            + (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

               float ld2 = a012 / l12;

//                // TODO: Why writing to a variable that's never read? Maybe it should be used afterwards?
//                pointProj = pointSel;
//                pointProj.x -= la * ld2;
//                pointProj.y -= lb * ld2;
//                pointProj.z -= lc * ld2;
               //根据距离设置权重
               float s = 1 - 0.9f * fabs(ld2);

               coeff.x = s * la;
               coeff.y = s * lb;
               coeff.z = s * lc;
               coeff.intensity = s * ld2;

               if (s > 0.1)
               {
                  _laserCloudOri.push_back(pointOri);
                  _coeffSel.push_back(coeff);
               }
            }
         }
      }

      //处理planar point
      for (int i = 0; i < laserCloudSurfStackNum; i++)
      {
         pointOri = _laserCloudSurfStackDS->points[i];
         pointAssociateToMap(pointOri, pointSel);
         kdtreeSurfFromMap.nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

         //此处和论文中以及上面通过构建协方差求edge line不一样，使用的是最小二乘法拟合平面，求planar patch的平面方程Ax+By+Cz+1=0
         if (pointSearchSqDis[4] < 1.0)
         {
            for (int j = 0; j < 5; j++)
            {//matA0(5,3)——5行3列矩阵
               matA0(j, 0) = _laserCloudSurfFromMap->points[pointSearchInd[j]].x;
               matA0(j, 1) = _laserCloudSurfFromMap->points[pointSearchInd[j]].y;
               matA0(j, 2) = _laserCloudSurfFromMap->points[pointSearchInd[j]].z;
            }
            //5个点构建平面方程，求参数A、B、C，联立得matA0*matX0=matB0
            matX0 = matA0.colPivHouseholderQr().solve(matB0);//矩阵matB0(5,1)，元素全设为1，对应将上面的平面的1移到等式右边

            float pa = matX0(0, 0);
            float pb = matX0(1, 0);
            float pc = matX0(2, 0);
            float pd = 1;

            float ps = sqrt(pa * pa + pb * pb + pc * pc);
            pa /= ps;
            pb /= ps;
            pc /= ps;
            pd /= ps;

            bool planeValid = true;
            for (int j = 0; j < 5; j++)
            {
               if (fabs(pa * _laserCloudSurfFromMap->points[pointSearchInd[j]].x +
                        pb * _laserCloudSurfFromMap->points[pointSearchInd[j]].y +
                        pc * _laserCloudSurfFromMap->points[pointSearchInd[j]].z + pd) > 0.2)
               {//将5个临近点带入平面方程检验平面方程是否拟合良好
                  planeValid = false;
                  break;
               }
            }

            if (planeValid)
            {
               float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

               //                // TODO: Why writing to a variable that's never read? Maybe it should be used afterwards?
               //                pointProj = pointSel;
               //                pointProj.x -= pa * pd2;
               //                pointProj.y -= pb * pd2;
               //                pointProj.z -= pc * pd2;

               float s = 1 - 0.9f * fabs(pd2) / sqrt(calcPointDistance(pointSel));

               coeff.x = s * pa;
               coeff.y = s * pb;
               coeff.z = s * pc;
               coeff.intensity = s * pd2;

               if (s > 0.1)
               {
                  _laserCloudOri.push_back(pointOri);
                  _coeffSel.push_back(coeff);
               }
            }
         }
      }

      float srx = _transformTobeMapped.rot_x.sin();
      float crx = _transformTobeMapped.rot_x.cos();
      float sry = _transformTobeMapped.rot_y.sin();
      float cry = _transformTobeMapped.rot_y.cos();
      float srz = _transformTobeMapped.rot_z.sin();
      float crz = _transformTobeMapped.rot_z.cos();

      size_t laserCloudSelNum = _laserCloudOri.size();
      if (laserCloudSelNum < 50)//特征点大于50个才进行优化迭代
         continue;

      //这部分的迭代过程与LaserOdometry节点的迭代过程系统，都是使用高斯牛顿法
      Eigen::Matrix<float, Eigen::Dynamic, 6> matA(laserCloudSelNum, 6);
      Eigen::Matrix<float, 6, Eigen::Dynamic> matAt(6, laserCloudSelNum);
      Eigen::Matrix<float, 6, 6> matAtA;
      Eigen::VectorXf matB(laserCloudSelNum);
      Eigen::VectorXf matAtB;
      Eigen::VectorXf matX;

      for (int i = 0; i < laserCloudSelNum; i++)
      {
         pointOri = _laserCloudOri.points[i];
         coeff = _coeffSel.points[i];

         float arx = (crx*sry*srz*pointOri.x + crx * crz*sry*pointOri.y - srx * sry*pointOri.z) * coeff.x
            + (-srx * srz*pointOri.x - crz * srx*pointOri.y - crx * pointOri.z) * coeff.y
            + (crx*cry*srz*pointOri.x + crx * cry*crz*pointOri.y - cry * srx*pointOri.z) * coeff.z;

         float ary = ((cry*srx*srz - crz * sry)*pointOri.x
                      + (sry*srz + cry * crz*srx)*pointOri.y + crx * cry*pointOri.z) * coeff.x
            + ((-cry * crz - srx * sry*srz)*pointOri.x
               + (cry*srz - crz * srx*sry)*pointOri.y - crx * sry*pointOri.z) * coeff.z;

         float arz = ((crz*srx*sry - cry * srz)*pointOri.x + (-cry * crz - srx * sry*srz)*pointOri.y)*coeff.x
            + (crx*crz*pointOri.x - crx * srz*pointOri.y) * coeff.y
            + ((sry*srz + cry * crz*srx)*pointOri.x + (crz*sry - cry * srx*srz)*pointOri.y)*coeff.z;

         matA(i, 0) = arx;
         matA(i, 1) = ary;
         matA(i, 2) = arz;
         matA(i, 3) = coeff.x;
         matA(i, 4) = coeff.y;
         matA(i, 5) = coeff.z;
         matB(i, 0) = -coeff.intensity;
      }

      matAt = matA.transpose();
      matAtA = matAt * matA;
      matAtB = matAt * matB;
      matX = matAtA.colPivHouseholderQr().solve(matAtB);

      //退化场景判断与处理
      if (iterCount == 0)
      {
         Eigen::Matrix<float, 1, 6> matE;
         Eigen::Matrix<float, 6, 6> matV;
         Eigen::Matrix<float, 6, 6> matV2;

         Eigen::SelfAdjointEigenSolver< Eigen::Matrix<float, 6, 6> > esolver(matAtA);
         matE = esolver.eigenvalues().real();
         matV = esolver.eigenvectors().real();

         matV2 = matV;

         isDegenerate = false;
         float eignThre[6] = { 100, 100, 100, 100, 100, 100 };
         for (int i = 0; i < 6; i++)
         {
            if (matE(0, i) < eignThre[i])
            {
               for (int j = 0; j < 6; j++)
               {
                  matV2(i, j) = 0;
               }
               isDegenerate = true;
            }
            else
            {
               break;
            }
         }
         matP = matV.inverse() * matV2;
      }

      if (isDegenerate)
      {
         Eigen::Matrix<float, 6, 1> matX2(matX);
         matX = matP * matX2;
      }

      //更新位姿
      _transformTobeMapped.rot_x += matX(0, 0);
      _transformTobeMapped.rot_y += matX(1, 0);
      _transformTobeMapped.rot_z += matX(2, 0);
      _transformTobeMapped.pos.x() += matX(3, 0);
      _transformTobeMapped.pos.y() += matX(4, 0);
      _transformTobeMapped.pos.z() += matX(5, 0);

      float deltaR = sqrt(pow(rad2deg(matX(0, 0)), 2) +
                          pow(rad2deg(matX(1, 0)), 2) +
                          pow(rad2deg(matX(2, 0)), 2));
      float deltaT = sqrt(pow(matX(3, 0) * 100, 2) +
                          pow(matX(4, 0) * 100, 2) +
                          pow(matX(5, 0) * 100, 2));

      if (deltaR < _deltaRAbort && deltaT < _deltaTAbort)
         break;//旋转平移量足够小就停止迭代
   }
   transformUpdate();

   sprintf(s,"iterationCount:%lu/%lu,isDegenerate:%s",iterCount,_maxIterations,isDegenerate?"true":"false");
   strcpy(log,s);
}


} // end namespace loam
