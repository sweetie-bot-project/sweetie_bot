cmake_minimum_required(VERSION 2.9)
project(sweetie_bot_gait_generator)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS 
	message_generation
	roscpp
	rosbag
	actionlib
	tf2
	tf2_geometry_msgs
	tf2_ros
	kdl_parser
	kdl_conversions 
	eigen_conversions 
	xpp_states
	xpp_msgs
	sensor_msgs
	geometry_msgs
	actionlib_msgs
	visualization_msgs
	towr
	towr_ros
	sweetie_bot_control_msgs 
	sweetie_bot_orocos_misc
	std_srvs
)

# Python setup
catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  EndEffectorGoal.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SaveTrajectory.srv
  DisplayMarkers.srv
)

# Generate actions in the 'action' folder
add_action_files(
  FILES
  MoveBase.action
)

# Actually generate the language-specific message and service files
generate_messages(DEPENDENCIES std_msgs geometry_msgs actionlib_msgs)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sweetie_bot_gait_towr
  CATKIN_DEPENDS 
	roscpp
	rosbag
	actionlib
	tf2
	tf2_geometry_msgs
	tf2_ros
	kdl_parser
	kdl_conversions 
	eigen_conversions 
	xpp_states
	xpp_msgs
	sensor_msgs
	geometry_msgs
	actionlib_msgs
	visualization_msgs
	towr_ros
	sweetie_bot_control_msgs 
	sweetie_bot_orocos_misc
	std_srvs
  DEPENDS 
    towr 
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})

## Declare a C++ library

## Declare a C++ executable
     
add_executable(towr_gait_generator src/gait_generator.cpp src/towr_trajectory_visualizer.cpp src/rigid_body_inertia_calculator.cpp)
target_compile_features(towr_gait_generator PRIVATE cxx_std_17)
target_link_libraries(towr_gait_generator ${catkin_LIBRARIES})
add_dependencies(towr_gait_generator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Rename C++ executable without prefix
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS towr_gait_generator
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Test scripts for gait_generator.
install(DIRECTORY scripts
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
	USE_SOURCE_PERMISSIONS)

## Install python scripts
#catkin_install_python(
#   PROGRAMS
#		scripts/walk
#		scripts/trot_forward.py
#		scripts/walk_forward.py
#		scripts/turn_left.py
#		scripts/walk_forward_free.py
#		scripts/shift_left.py
#		scripts/walk_forward2.py
#		scripts/walk_backward.py
#		scripts/turn_right.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


#############
## Testing ##
#############

# Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_sweetie_bot_gait_towr.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
