cmake_minimum_required(VERSION 3.0.2)
project(sweetie_bot_joint_state_publisher)

find_package(catkin REQUIRED)
catkin_package()

catkin_python_setup()

catkin_install_python(PROGRAMS
  scripts/sweetie_bot_joint_state_publisher
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
