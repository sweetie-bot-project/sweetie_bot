cmake_minimum_required(VERSION 2.9)
project(sweetie_bot_text_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  actionlib_msgs
)


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  TextCommand.msg
  CompoundActionElement.msg
  CompoundAction.msg
  Detection.msg
  DetectionArray.msg
  SpeechVisualization.msg
  SoundEvent.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  Transcribe.srv
  Complete.srv
  CompleteSimple.srv
  CompleteRaw.srv
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  TextAction.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs)

install(DIRECTORY
  info
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

