cmake_minimum_required(VERSION 2.8.3)
project(sweetiebot_cv_lite)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs message_generation)

add_message_files(
  FILES
  Rect.msg
  AprilTag.msg
  Faces.msg
  AprilTags.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime rospy std_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS scripts/bridge_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
