#include <ros/ros.h>
#include "loam_velodyne/LaserMapping.h"


/** Main node entry point. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "laserMapping");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");

  loam::LaserMapping laserMapping;

  if (laserMapping.setup(node, privateNode))
  {
    // initialization successful
    laserMapping.spin();
  }
  else
  {
    ROS_INFO("laserMapping node initialization failde!");
  }

  return 0;
}
