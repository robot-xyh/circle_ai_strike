cmake_minimum_required(VERSION 3.16)
project(circle_ipc VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(circle_ipc
  src/param_block_shm.cpp
  src/strike_params_snapshot_shm.cpp
  src/strike_telemetry_shm.cpp
  src/debug_preview_shm.cpp)

target_include_directories(circle_ipc
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../debug/include>
    $<INSTALL_INTERFACE:include>)

target_compile_features(circle_ipc PUBLIC cxx_std_17)

# preview_overlay.hpp needs OpenCV includes
find_package(OpenCV QUIET COMPONENTS core)
if(OpenCV_FOUND)
  target_include_directories(circle_ipc PUBLIC ${OpenCV_INCLUDE_DIRS})
endif()

# circle_ipc depends on circle_debug_common for PreviewOverlayShmData
if(TARGET circle_debug_common)
  target_link_libraries(circle_ipc PUBLIC circle_debug_common)
endif()

install(TARGETS circle_ipc EXPORT circle_ipcTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
