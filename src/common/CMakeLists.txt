cmake_minimum_required(VERSION 2.8.3)
project(common)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  message_generation
  geometry_msgs
)

SET(CMAKE_CXX_FLAGS "-frounding-math -std=c++11")

add_service_files(
   FILES
   Environment.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
        CATKIN_DEPENDS cmake_modules nav_msgs roscpp std_msgs tf geometry_msgs
        INCLUDE_DIRS include/
        LIBRARIES EnvironmentClient PointClickSubscriber RobotPoseSubscriber
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(EnvironmentClient src/EnvironmentClient.cpp)
target_link_libraries(EnvironmentClient ${catkin_LIBRARIES})

add_library(PointClickSubscriber src/PointClickSubscriber.cpp)
target_link_libraries(PointClickSubscriber ${catkin_LIBRARIES})

add_library(RobotPoseSubscriber src/RobotPoseSubscriber.cpp)
target_link_libraries(RobotPoseSubscriber ${catkin_LIBRARIES})