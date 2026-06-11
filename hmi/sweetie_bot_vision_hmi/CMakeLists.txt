cmake_minimum_required(VERSION 3.0.2)
project(sweetie_bot_vision_hmi)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  visualization_msgs
  sweetie_bot_text_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs visualization_msgs sweetie_bot_text_msgs
)

catkin_install_python(PROGRAMS
  scripts/vision_marker_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
