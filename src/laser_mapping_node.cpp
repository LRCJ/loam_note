#include <ros/ros.h>
#include "loam_velodyne/LaserMapping.h"


/** Main node entry point. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "laserMapping");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");

  //默认最大迭代次数是10次，平移量和旋转量的允许误差量，源码默认值为0.05
  loam::LaserMapping laserMapping;

  float fParam;
  int iParam;
  if (privateNode.getParam("maxIterations", iParam))
  {
    if (iParam < 1)
    {
      ROS_ERROR("Invalid maxIterations parameter: %d (expected > 0)", iParam);
      return false;
    }
    else
    {
      laserMapping.setMaxIterations(iParam);
      ROS_INFO("laserMapping node set maxIterations: %d", iParam);
    }
  }

  if (privateNode.getParam("deltaTAbort", fParam))
  {
    if (fParam <= 0)
    {
      ROS_ERROR("Invalid deltaTAbort parameter: %f (expected > 0)", fParam);
      return false;
    }
    else
    {
      laserMapping.setDeltaTAbort(fParam);
      ROS_INFO("laserMapping node set deltaTAbort: %g", fParam);
    }
  }

  if (privateNode.getParam("deltaRAbort", fParam))
  {
    if (fParam <= 0)
    {
      ROS_ERROR("Invalid deltaRAbort parameter: %f (expected > 0)", fParam);
      return false;
    }
    else
    {
      laserMapping.setDeltaRAbort(fParam);
      ROS_INFO("laserMapping node set deltaRAbort: %g", fParam);
    }
  }

  if (laserMapping.setup(node, privateNode))
  {
    // initialization successful
    laserMapping.spin();
  }

  return 0;
}
