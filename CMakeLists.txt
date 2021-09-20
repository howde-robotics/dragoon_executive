cmake_minimum_required(VERSION 3.10.2)
project(dragoon_executive VERSION 0.0.1)
set(GCC_COVERAGE_COMPILE_FLAGS "-Ofast")
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++17 -pthread")


find_package(catkin REQUIRED COMPONENTS
roscpp
rospy
std_msgs
sensor_msgs
nav_msgs
rostest
rosunit
actionlib_msgs
dragoon_messages
)

catkin_package(CATKIN_DEPENDS
roscpp
rospy
std_msgs
sensor_msgs
nav_msgs
tf
actionlib_msgs
)

add_subdirectory(planning)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

# add executables from your source code, i.e. compile it
# .py code are not included here as they do not need to be compiled
