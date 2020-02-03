cmake_minimum_required(VERSION 2.8.3)
project(hw2_pack)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  sensor_msgs
)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hw2_pack
#  CATKIN_DEPENDS geometry_msgs rospy sensor_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

