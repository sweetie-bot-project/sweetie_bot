cmake_minimum_required(VERSION 2.9)
project(sweetie_bot_proto2_deploy)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sweetie_bot_config
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

#############
## Install ##
#############

# Scripts installation
install(DIRECTORY default torque_control rpi joint_trajectories
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
	
# Set rviz configs writable from user account
install(
    FILES default/joint_space_control.rviz default/moveit_control.rviz
    PERMISSIONS OWNER_WRITE OWNER_READ GROUP_WRITE GROUP_READ WORLD_WRITE WORLD_READ
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/default
)

# bash script
install(PROGRAMS scripts/store-joint-trajectories scripts/store-msgs
	  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
