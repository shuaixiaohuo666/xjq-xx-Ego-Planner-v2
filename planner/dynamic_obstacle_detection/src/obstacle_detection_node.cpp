#include <ros/ros.h>
#include "dynamic_obstacle_detection/obstacle_detector.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dynamic_obstacle_detection_node");
  
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  ROS_INFO("Starting Dynamic Obstacle Detection Node...");

  dynamic_obstacle_detection::ObstacleDetector detector(nh, nh_private);
  
  detector.run();

  return 0;
}
