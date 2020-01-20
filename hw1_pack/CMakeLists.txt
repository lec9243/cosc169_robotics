cmake_minimum_required(VERSION 2.8.3)
project(hw1_pack)


find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hmk1_pack
#  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


