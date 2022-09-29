cmake_minimum_required(VERSION 2.8.8)
# http://habrahabr.ru/post/181838/ 
project(sweetie_bot_eyes_2)

# enable C++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Tell CMake to run moc when necessary:
set(CMAKE_AUTOMOC ON)
# As moc files are generated in the binary dir, tell CMake
# to always look for includes there:
set(CMAKE_INCLUDE_CURRENT_DIR ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#find_package(Qt6 REQUIRED COMPONENTS Gui)
find_package(Qt5 COMPONENTS Core Widgets OpenGL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
     roslib
 # sensor_msgs
  sweetie_bot_text_msgs
)

# QT packages
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)
include_directories("src/")
include_directories("src/animators/")
include_directories("src/effects/")

SET(eye_2_SOURCES src/main.cpp
    src/mainwindow.cpp
    src/eye_widget.cpp
    src/eye_painter.cpp
    src/eye_lid_painter.cpp
    src/eye_lid_param_preparer.cpp
    src/eye_image_loader.cpp
    src/eye_animation_manager.cpp
    src/animators/eye_mover_normal.cpp
    src/animators/eye_mover_roll.cpp
    src/animators/lid_animator_default.cpp
    src/animators/lid_animation_blink.cpp
    src/animators/lid_amimation_roll.cpp
    src/animators/eye_sizer_normal.cpp
    src/animators/eye_sizer_limb_shake.cpp
    src/animators/lid_animation_close.cpp
    src/ros_connector.cpp
    src/ros_subscriber.cpp
    src/ros_publisher.cpp
    src/ros_msg_parser.cpp
    src/cleanexit.cpp)
#SET(motor_state_viewer_HEADERS src/motor_state_display.h src/connect_widget.h src/state_viewer_form.h src/joint_trajectory_data.h src/joint_trajectory_point_table_view.h src/joint_list_table_view.h)
#SET(eye_2_FORMS src/motor_control_widget.ui src/motor_detail_display.ui src/ros_publisher_widget.ui src/ros_subscriber_widget.ui src/ros_connect_widget.ui src/state_viewer_form.ui)
#SET (eye_2_RESOURCE resource.qrc)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
#  LIBRARIES sweetie_bot_eyes
#  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

## Declare a C++ library
# add_library(sweetie_bot_eyes
#   src/${PROJECT_NAME}/sweetie_bot_eyes.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(sweetie_bot_eyes ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(sweetie_bot_eyes_node src/sweetie_bot_eyes_node.cpp)
#qt5_wrap_ui(eye_2_FORMS_HDRS ${eye_2_FORMS})
#qt5_add_resources(eye_2_RCC ${eye_2_RESOURCE})

#link_libraries(GL glut)


add_executable(eye_2 ${eye_2_SOURCES})# ${eye_2_RCC} ${eye_2_FORMS_HDRS})
#add_executable(eye_2_test src/main_2.cpp src/mainwindow.cpp ${motor_state_viewer_RCC})


qt5_use_modules(eye_2 Widgets)
#qt5_use_modules(eye_2_test Widgets)

add_dependencies(eye_2
  ${catkin_EXPORTED_TARGETS}
)

#add_dependencies(eye_2_test
#  ${catkin_EXPORTED_TARGETS}
#)

target_link_libraries(eye_2
   ${catkin_LIBRARIES}
)
#target_link_libraries(eye_2 PRIVATE Qt6::Gui)
target_link_libraries(eye_2 PRIVATE Qt5::OpenGL)
#target_link_libraries(eye_2_test
#   ${catkin_LIBRARIES}
#)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(sweetie_bot_eyes_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(sweetie_bot_eyes_node
#   ${catkin_LIBRARIES}
# )

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

# Mark executables and/or libraries for installation
 install(TARGETS eye_2
 #  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

#install(TARGETS eye_2_test
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_sweetie_bot_eyes.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
