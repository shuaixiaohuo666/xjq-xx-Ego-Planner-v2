cmake_minimum_required(VERSION 3.0.2)
project(dynamic_obstacle_detection)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  pcl_ros
  pcl_conversions
  tf
  tf2
  tf2_ros
  tf2_geometry_msgs
  traj_utils
)

find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dynamic_obstacle_detection
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    std_msgs 
    sensor_msgs 
    nav_msgs 
    geometry_msgs
    pcl_ros
    pcl_conversions
    tf
    tf2
    tf2_ros
    tf2_geometry_msgs
    traj_utils
  DEPENDS PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/obstacle_detector.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

## Declare a C++ executable
add_executable(dynamic_obstacle_detection_node 
  src/obstacle_detection_node.cpp
)

target_link_libraries(dynamic_obstacle_detection_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

## Install
install(TARGETS ${PROJECT_NAME} dynamic_obstacle_detection_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
