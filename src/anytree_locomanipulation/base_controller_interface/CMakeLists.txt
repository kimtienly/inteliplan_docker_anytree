cmake_minimum_required(VERSION 3.0.2)
project(base_controller_interface)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  actionlib
  message_generation
  position_controller
)

catkin_python_setup()

## Declare service files
add_service_files(
  FILES
  GoPose.srv
)

## Generate messages (no custom msgs, just srv here)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs actionlib message_runtime
)

catkin_install_python(PROGRAMS
  scripts/base_controller_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Add this line near the bottom
install(DIRECTORY src/base_controller_interface/
        DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}/base_controller_interface
        FILES_MATCHING PATTERN "*.py")
