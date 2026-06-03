add_executable(bf_runtime_test
  bf_runtime_test.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../bf_flight/src/strike_controller_adapter.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../bf_flight_png/src/png_controller_adapter.cpp)

target_include_directories(bf_runtime_test PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../bf_flight/src
  ${CMAKE_CURRENT_SOURCE_DIR}/../bf_flight_png/src)

target_link_libraries(bf_runtime_test PRIVATE
  circle_bf_runtime
  circle::circle_strike_core
  circle_bf_common
  circle_debug_common
  circle_ipc
  circle_perception)

get_filename_component(CIRCLE_PILOT_CONFIG_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../config" ABSOLUTE)

find_package(yaml-cpp QUIET)
if(yaml-cpp_FOUND)
  target_link_libraries(bf_runtime_test PRIVATE yaml-cpp)
  target_compile_definitions(bf_runtime_test PRIVATE
    CIRCLE_STRIKE_HAS_YAML=1
    CIRCLE_PILOT_CONFIG_DIR="${CIRCLE_PILOT_CONFIG_DIR}")
endif()

add_test(NAME bf_runtime_test COMMAND bf_runtime_test)
