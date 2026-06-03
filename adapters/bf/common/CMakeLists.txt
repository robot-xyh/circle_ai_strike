cmake_minimum_required(VERSION 3.16)
project(circle_bf_common VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT TARGET circle_strike_core AND NOT TARGET circle::circle_strike_core)
  get_filename_component(_CIRCLE_CORE_INSTALL
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../core/build/install" ABSOLUTE)
  find_package(circle_strike_core REQUIRED CONFIG
    PATHS
      "${_CIRCLE_CORE_INSTALL}/lib/cmake/circle_strike_core"
    NO_DEFAULT_PATH)
endif()

add_library(circle_bf_common
  src/msp_client.cpp
  src/bf_rc_mapper.cpp
  src/bf_fc_adapters.cpp
  src/bf_flight_config_yaml.cpp
  src/logger.cpp)

target_include_directories(circle_bf_common
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(circle_bf_common PUBLIC circle::circle_strike_core)

find_package(yaml-cpp QUIET)
if(yaml-cpp_FOUND)
  target_link_libraries(circle_bf_common PUBLIC yaml-cpp)
  target_compile_definitions(circle_bf_common PUBLIC CIRCLE_STRIKE_HAS_YAML=1)
endif()

install(TARGETS circle_bf_common
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include)
