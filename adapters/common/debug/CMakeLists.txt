cmake_minimum_required(VERSION 3.16)
project(circle_debug VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV QUIET COMPONENTS core imgproc imgcodecs)
if(NOT TARGET circle_strike_core AND NOT TARGET circle::circle_strike_core)
  find_package(circle_strike_core QUIET CONFIG)
endif()
if(NOT circle_strike_core_FOUND AND
   NOT TARGET circle_strike_core AND
   NOT TARGET circle::circle_strike_core)
  get_filename_component(_CORE_INSTALL
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../core/build/install" ABSOLUTE)
  get_filename_component(_WS_CORE_INSTALL
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../../../install/circle_strike_core" ABSOLUTE)
  find_package(circle_strike_core QUIET CONFIG
    PATHS
      "${_CORE_INSTALL}/lib/cmake/circle_strike_core"
    NO_DEFAULT_PATH)
endif()

add_library(circle_debug_common
  src/preview_overlay.cpp
  src/pipeline_perf.cpp
  src/tracking_debug_series.cpp
  src/telemetry_schema.cpp
  src/strike_param_tune.cpp
  src/strike_png_param_tune.cpp)
target_include_directories(circle_debug_common
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
target_include_directories(circle_debug_common
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../perception/include>)
target_compile_features(circle_debug_common PUBLIC cxx_std_17)
if(OpenCV_FOUND)
  target_include_directories(circle_debug_common PUBLIC ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(circle_debug_common PUBLIC ${OpenCV_LIBS})
else()
  message(FATAL_ERROR "circle_debug_common requires OpenCV core/imgproc")
endif()
if(TARGET circle::circle_strike_core)
  target_link_libraries(circle_debug_common PUBLIC circle::circle_strike_core)
elseif(TARGET circle_strike_core)
  target_link_libraries(circle_debug_common PUBLIC circle_strike_core)
else()
  message(FATAL_ERROR "circle_debug_common requires circle_strike_core")
endif()

add_library(circle_debug STATIC
  src/debug_runtime.cpp
  src/h264_video_encoder.cpp)
target_include_directories(circle_debug
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
if(NOT TARGET circle_ipc)
  message(FATAL_ERROR "circle_debug requires circle_ipc; add_subdirectory(../ipc) first")
endif()
target_link_libraries(circle_debug
  PUBLIC
    circle_debug_common
    circle_ipc)
target_compile_features(circle_debug PUBLIC cxx_std_17)

find_package(PkgConfig QUIET)
set(CIRCLE_DEBUG_HAS_WEBRTC 0)
if(OpenCV_FOUND AND PkgConfig_FOUND)
  pkg_check_modules(GST_WEBRTC QUIET
    gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0
    gstreamer-sdp-1.0 gstreamer-webrtc-1.0)
  if(GST_WEBRTC_FOUND)
    set(CIRCLE_DEBUG_HAS_WEBRTC 1)
    add_library(circle_debug_webrtc STATIC
      src/webrtc_h264_sender.cpp)
    target_include_directories(circle_debug_webrtc PUBLIC
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${GST_WEBRTC_INCLUDE_DIRS})
    target_link_libraries(circle_debug_webrtc PUBLIC
      circle_ipc
      circle_debug
      ${OpenCV_LIBS}
      ${GST_WEBRTC_LIBRARIES})
    target_compile_definitions(circle_debug_webrtc PUBLIC BF_DEBUGD_HAS_WEBRTC=1)
  endif()
endif()

install(TARGETS circle_debug_common circle_debug
  EXPORT circle_debugTargets
  ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib)
if(TARGET circle_debug_webrtc)
  install(TARGETS circle_debug_webrtc
    EXPORT circle_debugTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib)
endif()
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY web/
  DESTINATION share/circle/debug/web)
