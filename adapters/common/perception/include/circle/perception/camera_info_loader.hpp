#pragma once

#include <string>

#include "circle/types/detection.hpp"

namespace circle::perception {

/** Load OpenCV-style camera_matrix from decxin_* yaml (no ROS). */
bool loadCameraIntrinsicsFromYaml(const std::string& path,
                                  circle::types::CameraIntrinsics& out,
                                  int* out_width = nullptr,
                                  int* out_height = nullptr);

}  // namespace circle::perception
