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

#include <cmath>
#include "loam_velodyne/BasicTransformMaintenance.h"


namespace loam
{

using std::sin;
using std::cos;
using std::asin;
using std::atan2;


void BasicTransformMaintenance::updateOdometry(double pitch, double yaw, double roll, double x, double y, double z)
{
   _transformSum[0] = pitch;
   _transformSum[1] = yaw;
   _transformSum[2] = roll;
   _transformSum[3] = x;
   _transformSum[4] = y;
   _transformSum[5] = z;
}

void BasicTransformMaintenance::updateMappingTransform(double pitch, double yaw, double roll, double x, double y, double z, double twist_rot_x, double twist_rot_y, double twist_rot_z, double twist_pos_x, double twist_pos_y, double twist_pos_z)
{
   _transformAftMapped[0] = pitch;
   _transformAftMapped[1] = yaw;
   _transformAftMapped[2] = roll;

   _transformAftMapped[3] = x;
   _transformAftMapped[4] = y;
   _transformAftMapped[5] = z;

   _transformBefMapped[0] = twist_rot_x;
   _transformBefMapped[1] = twist_rot_y;
   _transformBefMapped[2] = twist_rot_z;

   _transformBefMapped[3] = twist_pos_x;
   _transformBefMapped[4] = twist_pos_y;
   _transformBefMapped[5] = twist_pos_z;
}

void BasicTransformMaintenance::updateMappingTransform(Twist const& transformAftMapped, Twist const& transformBefMapped)
{
   updateMappingTransform( transformAftMapped.rot_x.rad(), transformAftMapped.rot_y.rad(), transformAftMapped.rot_z.rad(), 
                           transformAftMapped.pos.x(), transformAftMapped.pos.y(), transformAftMapped.pos.z(),
                           transformBefMapped.rot_x.rad(), transformBefMapped.rot_y.rad(), transformBefMapped.rot_z.rad(),
                           transformBefMapped.pos.x(), transformBefMapped.pos.y(), transformBefMapped.pos.z());
}

//odometry的运动估计和mapping矫正量融合之后得到的最终的位姿transformMapped
void BasicTransformMaintenance::transformAssociateToMap()
{
   //_transformSum位姿数据publish的频率取决于odometry节点的执行速度，由于该节点计算量不大，
   //因此激光雷达以10Hz输出的点云数据，odometry节点都能及时处理完并输出一个位姿(10Hz)；
   //_transformBefMapped/_transformAftMapped位姿数据publish的频率取决于mapping节点的执行速度，
   //由于计算量较大一般无法跟上激光雷达输出频率，导致处理完一帧点云数据后，下一帧或下几帧点云数据都被跳过，
   //但依然可以使用处理完的这一帧输出的修正量去修正odometry节点输出的10Hz位姿信息，最终得到一个较精准的位姿



   //进行两个操作1.transformInc=_transformBefMapped-_transformSum，2.Rtfs*transformInc
   //这里_transformBefMapped/_transformAftMapped和_transformSum的时间戳是不一样的，
   //更准确地说，假设_transformBefMapped/_transformAftMapped是mapping节点处理第k帧点云得到的优化前后位姿，
   //则_transformSum可能是odometry节点处理第k+2帧得到的位姿，根据mapping节点的代码可知，
   //_transformBefMapped是mapping节点处理第k帧点云时优化前的位姿，等于odometry节点处理完第k帧点云时的_transformSum，
   //两者的位置坐标相减，得到世界坐标系下的第k和k+2帧Lidar的相对位置(注意，这里几个位姿均基于世界坐标系)，
   //然后通过_transformSum变换到k+2帧点云结束时刻的Lidar坐标下，绕YXZ旋转
   //    | cΘy , 0 , sΘy |
   //RΘy=|  0  , 1 ,  0  |
   //    |-sΘy , 0 , cΘy |
   //绕y轴旋转(-transformSum[1]=Θy)
   float x1 = cos(_transformSum[1]) * (_transformBefMapped[3] - _transformSum[3])
            - sin(_transformSum[1]) * (_transformBefMapped[5] - _transformSum[5]);
   float y1 = _transformBefMapped[4] - _transformSum[4];
   float z1 = sin(_transformSum[1]) * (_transformBefMapped[3] - _transformSum[3])
            + cos(_transformSum[1]) * (_transformBefMapped[5] - _transformSum[5]);

   //    | 1 ,  0  ,   0  |
   //RΘx=| 0 , cΘx , -sΘx |
   //    | 0 , sΘx ,  cΘx |
   //绕x轴旋转（-transformSum[0]=Θx）
   float x2 = x1;
   float y2 = cos(_transformSum[0]) * y1 + sin(_transformSum[0]) * z1;
   float z2 = -sin(_transformSum[0]) * y1 + cos(_transformSum[0]) * z1;

   //    | cΘz , -sΘz , 0 |
   //RΘz=| sΘz ,  cΘz , 0 |
   //    |  0  ,   0  , 1 |
   //绕z轴旋转（-transformSum[2]=Θz）
   _transformIncre[3] = cos(_transformSum[2]) * x2 + sin(_transformSum[2]) * y2;
   _transformIncre[4] = -sin(_transformSum[2]) * x2 + cos(_transformSum[2]) * y2;
   _transformIncre[5] = z2;



   //该部分三个由欧拉角得到的旋转矩阵相乘，进行位姿修正
   //Rbc，即_transformSum，由odometry节点输出的10Hz的位姿
   //      | cbcy*cbcz+sbcx*sbcy*sbcz , sbcx*sbcy*cbcz-cbcy*sbcz , cbcx*sbcy |
   //Rbc = |      cbcx*sbcz           ,         cbcx*cbcz        ,   -sbcx   |
   //      | sbcx*cbcy*sbcz-sbcy*cbcz , sbcx*cbcy*cbcz+sbcy*sbcz , cbcy*cbcx |
   float sbcx = sin(_transformSum[0]);
   float cbcx = cos(_transformSum[0]);
   float sbcy = sin(_transformSum[1]);
   float cbcy = cos(_transformSum[1]);
   float sbcz = sin(_transformSum[2]);
   float cbcz = cos(_transformSum[2]);

   //Rbl，即_transformBefMapped，由mapping节点输出的之前某帧点云的优化前位姿
   //      | cbly*cblz+sblx*sbly*sblz , sblx*sbly*cblz-cbly*sblz , cblx*sbly |
   //Rbl = |      cblx*sblz           ,         cblx*cblz        ,   -sblx   |
   //      | sblx*cbly*sblz-sbly*cblz , sblx*cbly*cblz+sbly*sblz , cbly*cblx |
   //Rbl的转置
   //       | cbly*cblz+sblx*sbly*sblz , cblx*sblz , sblx*cbly*sblz-sbly*cblz |
   //RblT = | sblx*sbly*cblz-cbly*sblz , cblx*cblz , sblx*cbly*cblz+sbly*sblz |
   //       |        cblx*sbly         ,   -sblx   ,        cbly*cblx         |
   float sblx = sin(_transformBefMapped[0]);
   float cblx = cos(_transformBefMapped[0]);
   float sbly = sin(_transformBefMapped[1]);
   float cbly = cos(_transformBefMapped[1]);
   float sblz = sin(_transformBefMapped[2]);
   float cblz = cos(_transformBefMapped[2]);

   //Ral，即_transformAftMapped，由mapping节点输出的之前某帧点云的优化后位姿
   //      | caly*calz+salx*saly*salz , salx*saly*calz-caly*salz , calx*saly |
   //Ral = |      calx*salz           ,         calx*calz        ,   -salx   |
   //      | salx*caly*salz-saly*calz , salx*caly*calz+saly*salz , caly*calx |
   float salx = sin(_transformAftMapped[0]);
   float calx = cos(_transformAftMapped[0]);
   float saly = sin(_transformAftMapped[1]);
   float caly = cos(_transformAftMapped[1]);
   float salz = sin(_transformAftMapped[2]);
   float calz = cos(_transformAftMapped[2]);

   //R = Ral * RblT * Rbc，用之前某帧点云的优化量(优化前后位姿变化量)直接修正odometry节点根据当前点云帧估计的位姿
   //之所以可以这样做，是因为Rbc本身就是由一帧一帧点云初始/结束时刻的位姿变化量叠加起来的，mapping节点使用sweep to map
   //方法进行点云匹配可以得到更高精度的点云初始/结束时刻的位姿变化量，自然就需要用这个更高精度的去替代odometry节点
   //估计的，而且是针对Rbc左乘，即相对于Rbc的固定坐标系(也就是世界坐标系)
   float srx = -sbcx * (salx * sblx + calx*salz * cblx*sblz + calx*calz * cblx*cblz)
      - cbcx*sbcy * (calx*calz*(cbly*sblz - cblz * sblx*sbly)
         - calx*salz * (cbly*cblz + sblx * sbly*sblz)
         + salx * cblx*sbly)
      - cbcx*cbcy * (calx*salz * (cblz*sbly - cbly*sblx*sblz)
         - calx*calz * (sbly*sblz + cbly*cblz*sblx)
         + salx * cblx*cbly);
   _transformMapped[0] = -asin(srx);

   float srycrx = sbcx * (cblx*cblz*(caly*salz - calz * salx*saly)
      - cblx * sblz*(caly*calz + salx * saly*salz) + calx * saly*sblx)
      - cbcx * cbcy*((caly*calz + salx * saly*salz)*(cblz*sbly - cbly * sblx*sblz)
         + (caly*salz - calz * salx*saly)*(sbly*sblz + cbly * cblz*sblx)
         - calx * cblx*cbly*saly)
      + cbcx * sbcy*((caly*calz + salx * saly*salz)*(cbly*cblz + sblx * sbly*sblz)
         + (caly*salz - calz * salx*saly)*(cbly*sblz - cblz * sblx*sbly)
         + calx * cblx*saly*sbly);
   float crycrx = sbcx * (cblx*sblz*(calz*saly - caly * salx*salz)
      - cblx * cblz*(saly*salz + caly * calz*salx) + calx * caly*sblx)
      + cbcx * cbcy*((saly*salz + caly * calz*salx)*(sbly*sblz + cbly * cblz*sblx)
         + (calz*saly - caly * salx*salz)*(cblz*sbly - cbly * sblx*sblz)
         + calx * caly*cblx*cbly)
      - cbcx * sbcy*((saly*salz + caly * calz*salx)*(cbly*sblz - cblz * sblx*sbly)
         + (calz*saly - caly * salx*salz)*(cbly*cblz + sblx * sbly*sblz)
         - calx * caly*cblx*sbly);
   _transformMapped[1] = atan2(srycrx / cos(_transformMapped[0]),
      crycrx / cos(_transformMapped[0]));

   float srzcrx = (cbcz*sbcy - cbcy * sbcx*sbcz)*(calx*salz*(cblz*sbly - cbly * sblx*sblz)
      - calx * calz*(sbly*sblz + cbly * cblz*sblx)
      + cblx * cbly*salx)
      - (cbcy*cbcz + sbcx * sbcy*sbcz)*(calx*calz*(cbly*sblz - cblz * sblx*sbly)
         - calx * salz*(cbly*cblz + sblx * sbly*sblz)
         + cblx * salx*sbly)
      + cbcx * sbcz*(salx*sblx + calx * cblx*salz*sblz + calx * calz*cblx*cblz);
   float crzcrx = (cbcy*sbcz - cbcz * sbcx*sbcy)*(calx*calz*(cbly*sblz - cblz * sblx*sbly)
      - calx * salz*(cbly*cblz + sblx * sbly*sblz)
      + cblx * salx*sbly)
      - (sbcy*sbcz + cbcy * cbcz*sbcx)*(calx*salz*(cblz*sbly - cbly * sblx*sblz)
         - calx * calz*(sbly*sblz + cbly * cblz*sblx)
         + cblx * cbly*salx)
      + cbcx * cbcz*(salx*sblx + calx * cblx*salz*sblz + calx * calz*cblx*cblz);
   _transformMapped[2] = atan2(srzcrx / cos(_transformMapped[0]),crzcrx / cos(_transformMapped[0]));

   //得到修正后的位姿_transformMapped，再将之前的transformInc变换回世界坐标系
   //绕Z轴旋转
   x1 = cos(_transformMapped[2]) * _transformIncre[3] - sin(_transformMapped[2]) * _transformIncre[4];
   y1 = sin(_transformMapped[2]) * _transformIncre[3] + cos(_transformMapped[2]) * _transformIncre[4];
   z1 = _transformIncre[5];

   //绕X轴旋转
   x2 = x1;
   y2 = cos(_transformMapped[0]) * y1 - sin(_transformMapped[0]) * z1;
   z2 = sin(_transformMapped[0]) * y1 + cos(_transformMapped[0]) * z1;

   //绕Y轴旋转，这里还要被_transformAftMapped减去，得到最终相对世界坐标系的位置，
   //因为已经修正了姿态，此处要被_transformAftMapped减去，而不是被_transformBefMapped减去
   //最好画几个坐标进行示意，更方便理解
   _transformMapped[3] = _transformAftMapped[3] - (cos(_transformMapped[1]) * x2 + sin(_transformMapped[1]) * z2);
   _transformMapped[4] = _transformAftMapped[4] - y2;
   _transformMapped[5] = _transformAftMapped[5] - (-sin(_transformMapped[1]) * x2 + cos(_transformMapped[1]) * z2);
}


} // end namespace loam
