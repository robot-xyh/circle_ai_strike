#pragma once

#include <string>

#include "circle/perception/camera_source.hpp"

namespace circle::perception {

/** Load Profile C flat camera yaml (no ros__parameters). */
bool loadMppCameraConfigFromYaml(const std::string& path, MppCameraSourceConfig& out);

}  // namespace circle::perception
