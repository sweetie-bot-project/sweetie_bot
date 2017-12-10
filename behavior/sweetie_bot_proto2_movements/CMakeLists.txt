cmake_minimum_required(VERSION 2.8.3)
project(sweetie_bot_proto2_movements)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)


## catkin specific configuration ##
###################################
catkin_package()


#############
## Install ##
#############

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY 
	joint_trajectories
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

