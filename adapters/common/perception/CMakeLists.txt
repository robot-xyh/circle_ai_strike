cmake_minimum_required(VERSION 3.16)
project(circle_perception VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

get_filename_component(_CORE_INSTALL
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../core/build/install" ABSOLUTE)
if(NOT TARGET circle_strike_core AND NOT TARGET circle::circle_strike_core)
  find_package(circle_strike_core QUIET CONFIG)
endif()
if(NOT circle_strike_core_FOUND AND
   NOT TARGET circle_strike_core AND
   NOT TARGET circle::circle_strike_core)
  find_package(circle_strike_core REQUIRED CONFIG
    PATHS
      "${_CORE_INSTALL}/lib/cmake/circle_strike_core"
    NO_DEFAULT_PATH)
endif()

# --- MPP + RGA (RK3588) ---
set(CIRCLE_PERCEPTION_USE_MPP_RGA 0)
set(_lib_dirs "/usr/lib" "/usr/lib/aarch64-linux-gnu" "/lib/aarch64-linux-gnu" "/usr/local/lib")
set(_inc_dirs "/usr/include" "/usr/local/include" "/usr/local/include/mpp" "/usr/include/rockchip")
find_library(CIRCLE_MPP_LIB NAMES rockchip_mpp mpp PATHS ${_lib_dirs})
find_library(CIRCLE_RGA_LIB NAMES rga PATHS ${_lib_dirs})
find_path(CIRCLE_MPP_INCLUDE NAMES rockchip/rk_mpi.h rk_mpi.h mpp.h PATHS ${_inc_dirs})
find_path(CIRCLE_RGA_INCLUDE NAMES RgaUtils.h rga.h im2d.h PATHS ${_inc_dirs})
if(CIRCLE_MPP_LIB AND CIRCLE_MPP_INCLUDE AND CIRCLE_RGA_LIB AND CIRCLE_RGA_INCLUDE)
  find_package(OpenCV QUIET COMPONENTS core imgproc imgcodecs)
  if(OpenCV_FOUND)
    set(CIRCLE_PERCEPTION_USE_MPP_RGA 1)
    message(STATUS "circle_perception: MPP+RGA enabled")
  endif()
endif()

# --- RKNN ---
set(CIRCLE_PERCEPTION_USE_RKNN 0)
find_library(CIRCLE_RKNN_LIB NAMES rknnrt PATHS ${_lib_dirs})
find_path(CIRCLE_RKNN_INCLUDE NAMES rknn_api.h PATHS ${_inc_dirs})
if(CIRCLE_RKNN_LIB AND CIRCLE_RKNN_INCLUDE)
  set(CIRCLE_PERCEPTION_USE_RKNN 1)
  message(STATUS "circle_perception: RKNN enabled")
endif()

find_package(yaml-cpp QUIET)

set(CIRCLE_PERCEPTION_SOURCES
  src/camera_source.cpp
  src/vision_pipeline.cpp
  src/camera_info_loader.cpp
  src/camera_config_loader.cpp
  src/rknn_engine.cpp
  src/mpp_rga_pipeline.cpp
  src/zero_copy_frame_slot.cpp
  src/parallel_vision_pipeline.cpp
  src/zero_copy_perception_runtime.cpp
)

add_library(circle_perception ${CIRCLE_PERCEPTION_SOURCES})

target_include_directories(circle_perception
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_compile_definitions(circle_perception
  PUBLIC
    CIRCLE_PERCEPTION_USE_MPP_RGA=${CIRCLE_PERCEPTION_USE_MPP_RGA}
    CIRCLE_PERCEPTION_USE_RKNN=${CIRCLE_PERCEPTION_USE_RKNN}
  PRIVATE
    USE_MPP_RGA=${CIRCLE_PERCEPTION_USE_MPP_RGA}
    USE_RKNN=${CIRCLE_PERCEPTION_USE_RKNN})

if(CIRCLE_MPP_INCLUDE)
  target_include_directories(circle_perception PRIVATE ${CIRCLE_MPP_INCLUDE})
endif()
if(CIRCLE_RGA_INCLUDE)
  target_include_directories(circle_perception PRIVATE ${CIRCLE_RGA_INCLUDE})
endif()
if(CIRCLE_RKNN_INCLUDE)
  target_include_directories(circle_perception PRIVATE ${CIRCLE_RKNN_INCLUDE})
endif()

target_link_libraries(circle_perception
  PUBLIC circle::circle_strike_core)

if(CIRCLE_PERCEPTION_USE_MPP_RGA)
  target_include_directories(circle_perception PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(circle_perception PUBLIC
    ${CIRCLE_MPP_LIB} ${CIRCLE_RGA_LIB} ${OpenCV_LIBS})
endif()
if(CIRCLE_PERCEPTION_USE_RKNN)
  target_link_libraries(circle_perception PRIVATE ${CIRCLE_RKNN_LIB})
endif()
if(yaml-cpp_FOUND)
  target_link_libraries(circle_perception PRIVATE yaml-cpp)
  target_compile_definitions(circle_perception PRIVATE CIRCLE_PERCEPTION_HAS_YAML=1)
endif()

target_compile_features(circle_perception PUBLIC cxx_std_17)

install(TARGETS circle_perception
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
