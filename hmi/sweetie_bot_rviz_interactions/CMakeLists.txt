cmake_minimum_required(VERSION 2.8.3)
project(sweetie_bot_rviz_interactions)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  visualization_msgs
  tf
  interactive_markers
  sweetie_bot_control_msgs
  sweetie_bot_clop_generator
  actionlib
)

# QT packages
find_package(Qt5Widgets REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. 
# catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sweetie_bot_rviz_interactions
  CATKIN_DEPENDS geometry_msgs roscpp visualization_msgs tf interactive_markers sweetie_bot_control_msgs sweetie_bot_clop_generator actionlib
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/sweetie_bot_rviz_interactions.cpp
# )

## Add cmake target dependencies of the library
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(${PROJECT_NAME}_pose_marker src/pose_marker_main.cpp src/pose_marker.cpp src/limb_pose_marker.cpp src/stance_pose_marker.cpp)
add_executable(${PROJECT_NAME}_destination_marker src/destination_marker_main.cpp src/destination_marker.cpp)

## Rename C++ executable without prefix
set_target_properties(${PROJECT_NAME}_pose_marker PROPERTIES OUTPUT_NAME pose_marker PREFIX "")
set_target_properties(${PROJECT_NAME}_destination_marker PROPERTIES OUTPUT_NAME destination_marker PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_pose_marker ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_destination_marker ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_pose_marker ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_destination_marker ${catkin_LIBRARIES} ${Qt5Widgets_LIBRARIES})

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_pose_marker
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}_destination_marker
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES file
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

