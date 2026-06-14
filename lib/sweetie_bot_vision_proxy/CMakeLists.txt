cmake_minimum_required(VERSION 3.0.2)
project(sweetie_bot_vision_proxy)

## The C++ front transport + visualizer of the vision federation: gstreamer ingest + ring buffer +
## Boost.Beast WebSocket to the provider container + flat-JSON round-trip to the py3.10 tracker-fuser +
## draw/publish (/image_raw, /detections, /hmi/vision_skeletons, /vision_proxy/result_json). It replaces
## the old python gasket as the single ROS face.
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  cv_bridge
  image_transport
  sweetie_bot_text_msgs
)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(OpenCV REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-app-1.0)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs visualization_msgs
                 cv_bridge image_transport sweetie_bot_text_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GST_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
)

add_executable(vision_proxy_node src/vision_proxy_node.cpp)
target_link_libraries(vision_proxy_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  opencv_freetype
  ${GST_LIBRARIES}
  ${JSONCPP_LIBRARIES}
  OpenSSL::SSL
  OpenSSL::Crypto
  Threads::Threads
)

install(TARGETS vision_proxy_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY fonts/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/fonts)
