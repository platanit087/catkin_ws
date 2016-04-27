cmake_minimum_required(VERSION 2.8.3)
project(navigation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  common
)

SET(CMAKE_CXX_FLAGS "-frounding-math -std=c++11")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${common_INCLUDE_DIRS}
)

add_library(LowLevelControl src/LowLevelControl.cpp)
target_link_libraries(LowLevelControl ${catkin_LIBRARIES})

add_library(PotentialFields src/PotentialFields.cpp)
target_link_libraries(PotentialFields ${catkin_LIBRARIES})

add_executable(basic_motion src/basic_motion.cpp)
target_link_libraries(basic_motion ${catkin_LIBRARIES} ${common_LIBRARIES} LowLevelControl)

add_executable(MotionPotentialFildsCentroids src/MotionPotentialFildsCentroids.cpp)
target_link_libraries(MotionPotentialFildsCentroids ${catkin_LIBRARIES} ${common_LIBRARIES} PotentialFields)

add_executable(MotionPotentialFildsSensors src/MotionPotentialFildsSensors.cpp)
target_link_libraries(MotionPotentialFildsSensors ${catkin_LIBRARIES} ${common_LIBRARIES} PotentialFields)