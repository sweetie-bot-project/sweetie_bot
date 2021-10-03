cmake_minimum_required(VERSION 2.8.3)
project(sweetie_bot_battery_monitor)

find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS rospy sensor_msgs)

catkin_install_python(PROGRAMS scripts/battery_monitor
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
