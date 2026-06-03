cmake_minimum_required(VERSION 3.16)
project(circle_bf_runtime VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(circle_bf_runtime
  src/bf_control_host.cpp
  src/bf_gating.cpp
  src/bf_runtime_config_yaml.cpp)

target_include_directories(circle_bf_runtime
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(circle_bf_runtime
  PUBLIC
    circle::circle_strike_core
    circle_bf_common
    circle_ipc
    circle_perception
    circle_debug_common)

find_package(yaml-cpp QUIET)
if(yaml-cpp_FOUND)
  target_link_libraries(circle_bf_runtime PUBLIC yaml-cpp)
  target_compile_definitions(circle_bf_runtime PUBLIC CIRCLE_STRIKE_HAS_YAML=1)
endif()

install(TARGETS circle_bf_runtime
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
