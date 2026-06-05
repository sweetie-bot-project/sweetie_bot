cmake_minimum_required(VERSION 3.16)
project(transport_cpp_client CXX)

# Native (ROS-agnostic) C++ transport for the Sweetie Bot vision federation. Standalone CMake project
# (NOT a catkin package — no package.xml, so catkin_make skips it); build it directly:
#   cmake -S . -B build && cmake --build build -j
#   ./build/transport_latency_probe --server-url wss://127.0.0.1:8443/ws --insecure --gst-pipeline '...'
#
# Today this contains ONLY the transport BENCHMARK PROBE (transport_latency_probe): it speaks the real
# wire framing (FrameHeader+JPEG up / VRES down, perfusion/transport/protocol.py in the vision repo),
# wss, TCP_NODELAY, 1-frame backpressure -- but the downlink is schema-blind (no msgpack decode, no
# fusion handoff, no reconnect). The PRODUCTION native client (decode PerceptionFrame -> hand to the
# native FusionDriver, reconnect, run-forever) is FUTURE WORK and belongs here too.
#
# Moved out of the vision repo (was sweetie_bot_vision/transport_cpp_client/) in the 2026-06
# consolidation: native C++ transport lives in the main sweetie_bot repo, not the vision repo.
# Deps: Boost.Beast + OpenSSL + GStreamer (no OpenCV / no jsoncpp / no ROS).

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED COMPONENTS system thread)
find_package(OpenSSL REQUIRED)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-app-1.0)

add_executable(transport_latency_probe transport_latency_probe.cpp)
target_include_directories(transport_latency_probe PRIVATE ${Boost_INCLUDE_DIRS} ${GST_INCLUDE_DIRS})
target_link_directories(transport_latency_probe PRIVATE ${GST_LIBRARY_DIRS})
target_link_libraries(transport_latency_probe PRIVATE
  ${Boost_LIBRARIES} ${GST_LIBRARIES} OpenSSL::SSL OpenSSL::Crypto Threads::Threads)
