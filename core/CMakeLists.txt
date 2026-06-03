cmake_minimum_required(VERSION 3.16)
project(circle_strike_core VERSION 0.1.0 LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake QUIET)
find_package(Eigen3 REQUIRED)

option(CIRCLE_STRIKE_BUILD_YAML "Build yaml-cpp param loader" ON)

set(CIRCLE_STRIKE_CORE_SOURCES
  src/strike/delayed_pixel_kalman.cpp
  src/strike/strike_controller.cpp
  src/strike/strike_params_clamp.cpp
  src/strike/modules/rate_shaper.cpp
  src/strike/modules/tilt_envelope.cpp
  src/strike/modules/yaw_controller.cpp
  src/strike/modules/edge_protection.cpp
  src/strike/modules/final_approach_gate.cpp
  src/strike/modules/speed_governor.cpp
  src/strike/modules/preclimb_module.cpp
  src/strike/modules/thrust_manager.cpp
  src/strike/modules/commit_module.cpp
  src/strike/modules/tilt_guard.cpp
  src/strike/modules/directional_dive.cpp
  src/strike/modules/terminal_predictor.cpp
  src/strike/modules/visual_png_guidance.cpp
  src/strike_png/visual_png_guidance.cpp
  src/strike_png/strike_png_controller.cpp
  src/vision/yolo_postprocess.cpp
)

if(CIRCLE_STRIKE_BUILD_YAML)
  find_package(yaml-cpp QUIET)
  if(yaml-cpp_FOUND)
    list(APPEND CIRCLE_STRIKE_CORE_SOURCES src/strike/strike_params_yaml.cpp)
    list(APPEND CIRCLE_STRIKE_CORE_SOURCES src/strike_png/strike_png_params_yaml.cpp)
  else()
    message(STATUS "circle_strike_core: yaml-cpp not found; param yaml loader disabled")
  endif()
endif()

add_library(circle_strike_core ${CIRCLE_STRIKE_CORE_SOURCES})

target_include_directories(circle_strike_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(circle_strike_core PUBLIC Eigen3::Eigen)

if(yaml-cpp_FOUND)
  target_link_libraries(circle_strike_core PUBLIC yaml-cpp)
  target_compile_definitions(circle_strike_core PUBLIC CIRCLE_STRIKE_HAS_YAML=1)
endif()

target_compile_features(circle_strike_core PUBLIC cxx_std_17)

install(TARGETS circle_strike_core
  EXPORT circle_strike_coreTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include)

install(EXPORT circle_strike_coreTargets
  FILE circle_strike_coreTargets.cmake
  NAMESPACE circle::
  DESTINATION lib/cmake/circle_strike_core)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/circle_strike_coreConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion)

configure_package_config_file(
  circle_strike_coreConfig.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/circle_strike_coreConfig.cmake"
  INSTALL_DESTINATION lib/cmake/circle_strike_core)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/circle_strike_coreConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/circle_strike_coreConfigVersion.cmake"
  DESTINATION lib/cmake/circle_strike_core)

if(ament_cmake_FOUND)
  ament_export_include_directories(include)
  ament_export_libraries(circle_strike_core)
  ament_export_dependencies(Eigen3)
  if(yaml-cpp_FOUND)
    ament_export_dependencies(yaml-cpp)
  endif()
  ament_package()
endif()

option(CIRCLE_STRIKE_BUILD_TESTS "Build circle_strike_core smoke tests" OFF)
if(CIRCLE_STRIKE_BUILD_TESTS)
  enable_testing()
  add_executable(circle_strike_core_smoke_test tests/smoke_test.cpp)
  target_link_libraries(circle_strike_core_smoke_test PRIVATE circle_strike_core)
  add_test(NAME circle_strike_core_smoke_test COMMAND circle_strike_core_smoke_test)

  # BF YAML parameter parse-coverage test (requires the yaml-cpp loader).
  if(yaml-cpp_FOUND)
    if(TARGET circle_strike_core AND NOT TARGET circle::circle_strike_core)
      add_library(circle::circle_strike_core ALIAS circle_strike_core)
    endif()
    if(NOT TARGET circle_bf_common)
      add_subdirectory(
        ${CMAKE_CURRENT_SOURCE_DIR}/../adapters/bf/common
        ${CMAKE_BINARY_DIR}/circle_bf_common)
    endif()
    add_executable(bf_params_parse_test tests/bf_params_parse_test.cpp)
    target_link_libraries(bf_params_parse_test PRIVATE
      circle_strike_core circle_bf_common)
    target_compile_definitions(bf_params_parse_test PRIVATE
      STRIKE_BF_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../config/strike_bf_flight.yaml")
    add_test(NAME bf_params_parse_test COMMAND bf_params_parse_test)
  endif()
endif()
