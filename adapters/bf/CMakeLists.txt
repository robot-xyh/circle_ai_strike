cmake_minimum_required(VERSION 3.16)
project(circle_bf VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT TARGET circle_strike_core AND NOT TARGET circle::circle_strike_core)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../core
    ${CMAKE_BINARY_DIR}/circle_strike_core)
endif()
if(TARGET circle_strike_core AND NOT TARGET circle::circle_strike_core)
  add_library(circle::circle_strike_core ALIAS circle_strike_core)
endif()

if(NOT TARGET circle_bf_common)
  add_subdirectory(common)
endif()
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../common/ipc
  ${CMAKE_BINARY_DIR}/circle_ipc)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../common/debug
  ${CMAKE_BINARY_DIR}/circle_debug)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../common/perception
  ${CMAKE_BINARY_DIR}/circle_perception)

add_subdirectory(bf_runtime)

add_executable(bf_flight
  bf_flight/src/main.cpp
  bf_flight/src/strike_controller_adapter.cpp)
target_link_libraries(bf_flight PRIVATE
  circle_bf_runtime
  circle::circle_strike_core
  circle_bf_common
  circle_debug_common
  circle_ipc
  circle_perception)

add_executable(bf_flight_png
  bf_flight_png/src/main.cpp
  bf_flight_png/src/png_controller_adapter.cpp)
target_link_libraries(bf_flight_png PRIVATE
  circle_bf_runtime
  circle::circle_strike_core
  circle_bf_common
  circle_debug_common
  circle_ipc
  circle_perception)

find_package(OpenCV QUIET COMPONENTS core imgproc imgcodecs)
add_executable(bf_debugd bf_debugd/src/main.cpp)
target_link_libraries(bf_debugd PRIVATE
  circle_bf_common
  circle::circle_strike_core
  circle_ipc
  circle_debug_common
  circle_debug)
if(OpenCV_FOUND)
  target_link_libraries(bf_debugd PRIVATE ${OpenCV_LIBS})
  target_include_directories(bf_debugd PRIVATE ${OpenCV_INCLUDE_DIRS})
endif()
if(TARGET circle_debug_webrtc)
  target_link_libraries(bf_debugd PRIVATE circle_debug_webrtc)
  target_compile_definitions(bf_debugd PRIVATE BF_DEBUGD_HAS_WEBRTC=1)
endif()

find_package(yaml-cpp QUIET)
if(yaml-cpp_FOUND)
  target_link_libraries(bf_flight PRIVATE yaml-cpp)
  target_compile_definitions(bf_flight PRIVATE CIRCLE_STRIKE_HAS_YAML=1)
  target_link_libraries(bf_flight_png PRIVATE yaml-cpp)
  target_compile_definitions(bf_flight_png PRIVATE CIRCLE_STRIKE_HAS_YAML=1)
  target_link_libraries(bf_debugd PRIVATE yaml-cpp)
  target_compile_definitions(bf_debugd PRIVATE CIRCLE_STRIKE_HAS_YAML=1)
  target_link_libraries(circle_debug_common PUBLIC yaml-cpp)
  target_compile_definitions(circle_debug_common PUBLIC CIRCLE_STRIKE_HAS_YAML=1)
endif()

install(TARGETS bf_flight bf_flight_png bf_debugd RUNTIME DESTINATION bin)

option(CIRCLE_BF_BUILD_TESTS "Build Betaflight adapter unit/smoke tests" OFF)
if(CIRCLE_BF_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
