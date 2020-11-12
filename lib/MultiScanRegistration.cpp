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

#include "loam_velodyne/MultiScanRegistration.h"

namespace loam {

MultiScanMapper::MultiScanMapper(const float& lowerBound,
                                 const float& upperBound,
                                 const uint16_t& nScanRings)
    : _lowerBound(lowerBound),
      _upperBound(upperBound),
      _nScanRings(nScanRings),
      _factor((nScanRings - 1) / (upperBound - lowerBound))
{}

void MultiScanMapper::set(const float &lowerBound,
                          const float &upperBound,
                          const uint16_t &nScanRings)
{
  _lowerBound = lowerBound;
  _upperBound = upperBound;
  _nScanRings = nScanRings;
  _factor = (nScanRings - 1) / (upperBound - lowerBound);
}

int MultiScanMapper::getRingForAngle(const float& angle) {
  return int(((angle * 180 / M_PI) - _lowerBound) * _factor + 0.5);
}



MultiScanRegistration::MultiScanRegistration(const MultiScanMapper& scanMapper)
    : _scanMapper(scanMapper)//在MultiScanRegigtration.h里声明该构造函数时已经给scanMapper赋予了默认参数
{}

bool MultiScanRegistration::setup(ros::NodeHandle& node, ros::NodeHandle& privateNode)
{
  RegistrationParams config;
  // subscribe to IMU topic
  _subImu = node.subscribe<sensor_msgs::Imu>("/imu/data", 50, &MultiScanRegistration::handleIMUMessage, this);

  // advertise scan registration topics
  _pubLaserCloud            = node.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 2);
  _pubCornerPointsSharp     = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 2);
  _pubCornerPointsLessSharp = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 2);
  _pubSurfPointsFlat        = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 2);
  _pubSurfPointsLessFlat    = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 2);
  _pubImuTrans              = node.advertise<sensor_msgs::PointCloud2>("/imu_trans", 5);

  // fetch scan mapping params
  std::string lidarName;
  std::string topicName;

  if (privateNode.getParam("lidar", lidarName))
  {
    if (lidarName == "VLP-16")
      _scanMapper = MultiScanMapper::Velodyne_VLP_16();
    else if (lidarName == "HDL-32")
      _scanMapper = MultiScanMapper::Velodyne_HDL_32();
    else if (lidarName == "HDL-64E")
      _scanMapper = MultiScanMapper::Velodyne_HDL_64E();
    else if (lidarName == "PandarQT")
      _scanMapper = MultiScanMapper::PandarQT();
    else
    {
      ROS_ERROR("Invalid lidar parameter: %s (only \"VLP-16\", \"HDL-32\" , \"HDL-64E\" and \"PandarQT\" are supported)!", lidarName.c_str());
      return false;
    }
    ROS_INFO("multiScanRegistration node set %s scan mapper!", lidarName.c_str());
  }
  else
  {/*
    float vAngleMin, vAngleMax;
    int nScanRings;

    if (privateNode.getParam("minVerticalAngle", vAngleMin) &&
        privateNode.getParam("maxVerticalAngle", vAngleMax) &&
        privateNode.getParam("nScanRings", nScanRings)) {
      if (vAngleMin >= vAngleMax) {
        ROS_ERROR("Invalid vertical range (min >= max)");
        return false;
      } else if (nScanRings < 2) {
        ROS_ERROR("Invalid number of scan rings (n < 2)");
        return false;
      }

      _scanMapper.set(vAngleMin, vAngleMax, nScanRings);
      ROS_INFO("Set linear scan mapper from %g to %g degrees with %d scan rings.", vAngleMin, vAngleMax, nScanRings);
    }*/
    ROS_ERROR("Please enter the Lidar Name!");
  }

  // subscribe to input cloud topic
  if (privateNode.getParam("PointCloudTopicName", topicName))
  {
    _subLaserCloud = node.subscribe<sensor_msgs::PointCloud2>
        (topicName, 2, &MultiScanRegistration::handleCloudMessage, this);
    ROS_INFO("laserMapping node set maxIterations: %s", topicName.c_str());
  }

  parseParams(privateNode,config);
  configure(config);

  return true;
}



void MultiScanRegistration::handleCloudMessage(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
  if (_systemDelay > 0) 
  {
    --_systemDelay;
    return;
  }

  // fetch new input cloud
  pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
  pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);

  process(laserCloudIn, fromROSTime(laserCloudMsg->header.stamp));
}
void MultiScanRegistration::handleIMUMessage(const sensor_msgs::Imu::ConstPtr& imuIn)
{
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(imuIn->orientation, orientation);
  double roll, pitch, yaw;
  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

  Vector3 acc;
  acc.x() = float(imuIn->linear_acceleration.y - sin(roll) * cos(pitch) * 9.81);
  acc.y() = float(imuIn->linear_acceleration.z - cos(roll) * cos(pitch) * 9.81);
  acc.z() = float(imuIn->linear_acceleration.x + sin(pitch)             * 9.81);

  IMUState newState;
  newState.stamp = fromROSTime( imuIn->header.stamp);
  newState.roll = roll;
  newState.pitch = pitch;
  newState.yaw = yaw;
  newState.acceleration = acc;

//  updateIMUData(acc, newState);
}


void MultiScanRegistration::process(const pcl::PointCloud<pcl::PointXYZ>& laserCloudIn, const Time& scanTime)
{
  size_t cloudSize = laserCloudIn.size();

  // determine scan start and end orientations
  float startOri = -std::atan2(laserCloudIn[0].y, laserCloudIn[0].x);
  float endOri = -std::atan2(laserCloudIn[cloudSize - 1].y,
                             laserCloudIn[cloudSize - 1].x) + 2 * float(M_PI);
  if (endOri - startOri > 3 * M_PI)
    endOri -= 2 * M_PI;
  else if (endOri - startOri < M_PI)
    endOri += 2 * M_PI;

  bool halfPassed = false;
  pcl::PointXYZI point;
  _laserCloudScans.resize(_scanMapper.getNumberOfScanRings());
  // clear all scanline points
  std::for_each(_laserCloudScans.begin(), _laserCloudScans.end(), [](auto&&v) {v.clear(); }); 

  // extract valid points from input cloud
  for (int i = 0; i < cloudSize; i++)
  {
    point.x = laserCloudIn[i].y;
    point.y = laserCloudIn[i].z;
    point.z = laserCloudIn[i].x;

    // skip NaN and INF valued points
    if (!pcl_isfinite(point.x) || !pcl_isfinite(point.y) || !pcl_isfinite(point.z))
      continue;

    // skip zero valued points
    if (point.x * point.x + point.y * point.y + point.z * point.z < 0.0001)
      continue;

    // calculate vertical point angle and scan ID
    float angle = std::atan(point.y / std::sqrt(point.x * point.x + point.z * point.z));
    int scanID = _scanMapper.getRingForAngle(angle);
    if (scanID >= _scanMapper.getNumberOfScanRings() || scanID < 0 )
      continue;

    // calculate horizontal point angle
    float ori = -std::atan2(point.x, point.z);
    if (!halfPassed)
    {
      if (ori < startOri - M_PI / 2)
        ori += 2 * M_PI;
      else if (ori > startOri + M_PI * 3 / 2)
        ori -= 2 * M_PI;
      if (ori - startOri > M_PI)
        halfPassed = true;
    }
    else
    {
      ori += 2 * M_PI;

      if (ori < endOri - M_PI * 3 / 2)
        ori += 2 * M_PI;
      else if (ori > endOri + M_PI / 2)
        ori -= 2 * M_PI;
    }

    // calculate relative scan time based on point orientation
    float relTime = config().scanPeriod * (ori - startOri) / (endOri - startOri);
    point.intensity = scanID + relTime;

    //projectPointToStartOfSweep(point, relTime);//可注释掉

    _laserCloudScans[scanID].push_back(point);
  }

  processScanlines(scanTime, _laserCloudScans);
  publishResult();
}

void MultiScanRegistration::publishResult()
{
  auto sweepStartTime = toROSTime(sweepStart());
  // publish full resolution and feature point clouds
  publishCloudMsg(_pubLaserCloud, laserCloud(), sweepStartTime, "/camera");
  publishCloudMsg(_pubCornerPointsSharp, cornerPointsSharp(), sweepStartTime, "/camera");
  publishCloudMsg(_pubCornerPointsLessSharp, cornerPointsLessSharp(), sweepStartTime, "/camera");
  publishCloudMsg(_pubSurfPointsFlat, surfacePointsFlat(), sweepStartTime, "/camera");
  publishCloudMsg(_pubSurfPointsLessFlat, surfacePointsLessFlat(), sweepStartTime, "/camera");

  // publish corresponding IMU transformation information
  publishCloudMsg(_pubImuTrans, imuTransform(), sweepStartTime, "/camera");
}


bool MultiScanRegistration::parseParams(const ros::NodeHandle& nh, RegistrationParams& config_out) 
{
  bool success = true;
  int iParam = 0;
  float fParam = 0;

  if (nh.getParam("scanPeriod", fParam))
  {
    if (fParam <= 0)
    {
      ROS_ERROR("Invalid scanPeriod parameter: %f (expected > 0)", fParam);
      success = false;
    }
    else
    {
      config_out.scanPeriod = fParam;
      ROS_INFO("Set scanPeriod: %g", fParam);
    }
  }

  if (nh.getParam("imuHistorySize", iParam))
  {
    if (iParam < 1)
    {
      ROS_ERROR("Invalid imuHistorySize parameter: %d (expected >= 1)", iParam);
      success = false;
    }
    else
    {
      config_out.imuHistorySize = iParam;
      ROS_INFO("Set imuHistorySize: %d", iParam);
    }
  }

  if (nh.getParam("featureRegions", iParam))
  {
    if (iParam < 1)
    {
      ROS_ERROR("Invalid featureRegions parameter: %d (expected >= 1)", iParam);
      success = false;
    }
    else
    {
      config_out.nFeatureRegions = iParam;
      ROS_INFO("Set nFeatureRegions: %d", iParam);
    }
  }

  if (nh.getParam("curvatureRegion", iParam))
  {
    if (iParam < 1)
    {
      ROS_ERROR("Invalid curvatureRegion parameter: %d (expected >= 1)", iParam);
      success = false;
    }
    else
    {
      config_out.curvatureRegion = iParam;
      ROS_INFO("Set curvatureRegion: +/- %d", iParam);
    }
  }

  if (nh.getParam("maxCornerSharp", iParam))
  {
    if (iParam < 1)
    {
      ROS_ERROR("Invalid maxCornerSharp parameter: %d (expected >= 1)", iParam);
      success = false;
    }
    else
    {
      config_out.maxCornerSharp = iParam;
      config_out.maxCornerLessSharp = 10 * iParam;
      ROS_INFO("Set maxCornerSharp / less sharp: %d / %d", iParam, config_out.maxCornerLessSharp);
    }
  }

  if (nh.getParam("maxCornerLessSharp", iParam))
  {
    if (iParam < config_out.maxCornerSharp)
    {
      ROS_ERROR("Invalid maxCornerLessSharp parameter: %d (expected >= %d)", iParam, config_out.maxCornerSharp);
      success = false;
    }
    else
    {
      config_out.maxCornerLessSharp = iParam;
      ROS_INFO("Set maxCornerLessSharp: %d", iParam);
    }
  }

  if (nh.getParam("maxSurfaceFlat", iParam))
  {
    if (iParam < 1)
    {
      ROS_ERROR("Invalid maxSurfaceFlat parameter: %d (expected >= 1)", iParam);
      success = false;
    }
    else
    {
      config_out.maxSurfaceFlat = iParam;
      ROS_INFO("Set maxSurfaceFlat: %d", iParam);
    }
  }

  if (nh.getParam("surfaceCurvatureThreshold", fParam))
  {
    if (fParam < 0.001)
    {
      ROS_ERROR("Invalid surfaceCurvatureThreshold parameter: %f (expected >= 0.001)", fParam);
      success = false;
    }
    else
    {
      config_out.surfaceCurvatureThreshold = fParam;
      ROS_INFO("Set surfaceCurvatureThreshold: %g", fParam);
    }
  }

  if (nh.getParam("lessFlatFilterSize", fParam))
  {
    if (fParam < 0.001)
    {
      ROS_ERROR("Invalid lessFlatFilterSize parameter: %f (expected >= 0.001)", fParam);
      success = false;
    }
    else
    {
      config_out.lessFlatFilterSize = fParam;
      ROS_INFO("Set lessFlatFilterSize: %g", fParam);
    }
  }

  return success;
}


} // end namespace loam
