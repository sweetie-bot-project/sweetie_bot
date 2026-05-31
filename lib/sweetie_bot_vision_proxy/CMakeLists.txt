cmake_minimum_required(VERSION 3.0.2)
project(sweetie_bot_vision_proxy)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)


find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED
  gstreamer-1.0
  gstreamer-app-1.0
)

find_package(Boost REQUIRED COMPONENTS
  system
  thread
)

find_package(OpenSSL REQUIRED)

find_package(OpenCV REQUIRED)

pkg_check_modules(JSONCPP REQUIRED jsoncpp)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GST_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
)

link_directories(
  ${GST_LIBRARY_DIRS}
)

add_executable(vision_proxy_node
  src/vision_proxy_node.cpp
)

target_compile_options(vision_proxy_node PRIVATE
  ${GST_CFLAGS_OTHER}
)

target_link_libraries(vision_proxy_node
  ${catkin_LIBRARIES}
  ${GST_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${JSONCPP_LIBRARIES}
  OpenSSL::SSL
  OpenSSL::Crypto
)
