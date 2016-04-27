cmake_minimum_required(VERSION 2.8.3)
project(planning)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  nav_msgs
  std_msgs
  geometry_msgs
  actionlib_msgs
  visualization_msgs
  tf
  tf_conversions
  common
)

find_package(CGAL QUIET COMPONENTS Core )

SET(CMAKE_CXX_FLAGS "-frounding-math -std=c++11")

add_action_files(
        DIRECTORY action
        FILES GoToTask.action
)

generate_messages(
    DEPENDENCIES
    actionlib_msgs std_msgs geometry_msgs
)

catkin_package(
        CATKIN_DEPENDS message_runtime actionlib_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CGAL_INCLUDE_DIRS}
  ${common_INCLUDE_DIRS}
)

add_executable(GotoTaskAction src/GotoTaskAction.cpp)
target_link_libraries(GotoTaskAction ${catkin_LIBRARIES} ${common_LIBRARIES} ${CGAL_LIBRARY} ${GMP_LIBRARIES} )
add_dependencies(
  GotoTaskAction
  ${planning_EXPORTED_TARGETS}
)

add_executable(path_planning_test src/path_planning_test.cpp)
target_link_libraries(path_planning_test ${catkin_LIBRARIES})