cmake_minimum_required(VERSION 3.0.2)

project(position_controller)
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  roscpp
  pcl_conversions
  pcl_ros
  tf_conversions
  dynamic_reconfigure
  actionlib
  message_generation
  geometry_msgs
)


generate_dynamic_reconfigure_options(cfg/PositionController.cfg)

## Generate actions in the 'action' folder
add_action_files(
   FILES
   PositionController.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   geometry_msgs
   std_msgs  # Or other packages containing msgs
   actionlib_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS eigen_conversions roscpp
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)





######################################################
add_library(position_controller
  src/position_controller/position_controller.cpp
)

add_dependencies(position_controller ${catkin_EXPORTED_TARGETS})
target_link_libraries(position_controller ${catkin_LIBRARIES})


add_executable(position_controller_node
  src/position_controller/position_controller_node.cpp )
target_link_libraries(position_controller_node ${catkin_LIBRARIES} position_controller)
add_dependencies(position_controller_node ${PROJECT_NAME}_gencfg)


######################################################
#add_executable(simple_heading_controller_node
#  src/position_controller/simple_heading_controller_node.cpp )
#target_link_libraries(simple_heading_controller_node ${catkin_LIBRARIES} position_controller)


######################################################
add_executable(lidar_sweep_collector_node
  src/lidar_sweep_collector/lidar_sweep_collector_node.cpp)
target_link_libraries(lidar_sweep_collector_node ${catkin_LIBRARIES})


######################################################
add_executable(point_cloud_collector_node
  src/lidar_sweep_collector/point_cloud_collector_node.cpp)
target_link_libraries(point_cloud_collector_node ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS ${TARGETS}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
