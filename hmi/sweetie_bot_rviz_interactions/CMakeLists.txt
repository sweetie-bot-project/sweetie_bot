cmake_minimum_required(VERSION 2.9)
project(sweetie_bot_rviz_interactions)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  actionlib
  interactive_markers
  visualization_msgs
  geometry_msgs
  sweetie_bot_gait_generator
  sweetie_bot_control_msgs
  sweetie_bot_text_msgs
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
  CATKIN_DEPENDS roscpp tf actionlib interactive_markers visualization_msgs geometry_msgs sweetie_bot_gait_generator sweetie_bot_control_msgs sweetie_bot_text_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(${catkin_INCLUDE_DIRS})

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/sweetie_bot_rviz_interactions.cpp
# )

## Add cmake target dependencies of the library
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(${PROJECT_NAME}_robot_pose_marker src/robot_pose_marker_main.cpp src/pose_marker_base.cpp src/limb_pose_marker.cpp src/stance_pose_marker.cpp)
add_executable(${PROJECT_NAME}_generic_pose_marker src/generic_pose_marker_main.cpp src/pose_marker_base.cpp src/generic_pose_marker.cpp)
add_executable(${PROJECT_NAME}_destination_marker src/destination_marker_main.cpp src/destination_marker.cpp)
add_executable(${PROJECT_NAME}_object_detection_marker src/object_detection_marker_main.cpp src/object_detection_marker.cpp)

## Rename C++ executable without prefix
set_target_properties(${PROJECT_NAME}_robot_pose_marker PROPERTIES OUTPUT_NAME robot_pose_marker PREFIX "")
set_target_properties(${PROJECT_NAME}_generic_pose_marker PROPERTIES OUTPUT_NAME generic_pose_marker PREFIX "")
set_target_properties(${PROJECT_NAME}_destination_marker PROPERTIES OUTPUT_NAME destination_marker PREFIX "")
set_target_properties(${PROJECT_NAME}_object_detection_marker PROPERTIES OUTPUT_NAME object_detection_marker PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_robot_pose_marker ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_generic_pose_marker ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_destination_marker ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_object_detection_marker ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_robot_pose_marker ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_generic_pose_marker ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_destination_marker ${catkin_LIBRARIES} ${Qt5Widgets_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_object_detection_marker ${catkin_LIBRARIES})

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
install(TARGETS ${PROJECT_NAME}_robot_pose_marker
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}_generic_pose_marker
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}_destination_marker
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}_object_detection_marker
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

