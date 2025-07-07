cmake_minimum_required(VERSION 3.0.2)
project(anymal_base_control)

## Find required catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  geometry_msgs
  navigation_manager_msgs
)

## Declare catkin package
catkin_package(
  CATKIN_DEPENDS roscpp actionlib geometry_msgs navigation_manager_msgs
)

## Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Build your node
add_executable(navigate_to_goal_node src/send_goal_node.cpp)

target_link_libraries(navigate_to_goal_node
  ${catkin_LIBRARIES}
)
