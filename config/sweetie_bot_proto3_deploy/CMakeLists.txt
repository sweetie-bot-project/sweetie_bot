cmake_minimum_required(VERSION 2.8.3)
project(sweetie_bot_proto3_deploy)

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
install(DIRECTORY default joint_trajectories
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
# bash script
install(PROGRAMS scripts/store-joint-trajectories
	  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
