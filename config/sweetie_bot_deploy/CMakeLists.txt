cmake_minimum_required(VERSION 2.8.3)
project(sweetie_bot_deploy)

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
install(DIRECTORY highlevel
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY control
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY motion_core
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY common
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
# python scripts installation
catkin_install_python(PROGRAMS
    scripts/execute-joint-trajectory
    scripts/execute-step-sequence
    scripts/servo-identification
    scripts/servos_cpf_generator.pl
    scripts/servos_models_cpf_generator.pl
    scripts/set-base-link-height.py
    scripts/set-named-joints-pose.py
    scripts/step-sequence-strip
    scripts/step-sequence-strip-supports
    scripts/store
    scripts/update_servo_offsets
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
