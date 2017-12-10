cmake_minimum_required(VERSION 2.8.3)
project(velodyne_slam)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES velodyne_slam
#  CATKIN_DEPENDS geometry_msgs pcl_conversions pcl_ros roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(lidarMapping src/lidarMapping.cpp)
target_link_libraries(lidarMapping ${catkin_LIBRARIES})