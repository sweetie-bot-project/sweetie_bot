cmake_minimum_required(VERSION 3.6)
project(sweetie_bot_voice)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  rospack
  sound_play
  sweetie_bot_text_msgs
  sweetie_bot_load_balancer
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS src
#  LIBRARIES ${PROJECT_NAME}
)


#############
## Install ##
#############
install(DIRECTORY
  launch
  config
  conversion
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
                      scripts/voice
                      scripts/logger
                      scripts/tts_api_service_node
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# install shell scripts
#install(DIRECTORY scripts
#	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#	USE_SOURCE_PERMISSIONS)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
#install(DIRECTORY launch sounds
#    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

