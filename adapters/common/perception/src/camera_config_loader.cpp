#include "circle/perception/camera_config_loader.hpp"

#include <fstream>
#include <regex>

namespace circle::perception {

namespace {

uint32_t parseUint(const std::string& yaml, const std::string& key, uint32_t fallback) {
  const std::regex re(key + R"(\s*:\s*([0-9]+))");
  std::smatch m;
  if (std::regex_search(yaml, m, re) && m.size() > 1) {
    return static_cast<uint32_t>(std::stoul(m[1].str()));
  }
  return fallback;
}

/**
 * Parse signed int (with optional `-`); returns fallback when key absent.
 * Match the same line scope used by parseUint so comment-only mentions
 * (e.g. "# old default contrast: 70 ...") are still tolerated.
 */
int parseInt(const std::string& yaml, const std::string& key, int fallback) {
  const std::regex re(key + R"(\s*:\s*(-?[0-9]+))");
  std::smatch m;
  if (std::regex_search(yaml, m, re) && m.size() > 1) {
    return std::stoi(m[1].str());
  }
  return fallback;
}

bool parseBool(const std::string& yaml, const std::string& key, bool fallback) {
  const std::regex re(key + R"(\s*:\s*(true|false|True|False|TRUE|FALSE|1|0))");
  std::smatch m;
  if (std::regex_search(yaml, m, re) && m.size() > 1) {
    const std::string v = m[1].str();
    if (v == "true" || v == "True" || v == "TRUE" || v == "1") {
      return true;
    }
    return false;
  }
  return fallback;
}

std::string parseString(const std::string& yaml, const std::string& key,
                        const std::string& fallback) {
  const std::regex re(key + R"_(\s*:\s*"?([A-Za-z0-9_./:-]+)"?)_");
  std::smatch m;
  if (std::regex_search(yaml, m, re) && m.size() > 1) {
    return m[1].str();
  }
  return fallback;
}

}  // namespace

bool loadMppCameraConfigFromYaml(const std::string& path, MppCameraSourceConfig& out) {
  std::ifstream in(path);
  if (!in) {
    return false;
  }
  const std::string yaml((std::istreambuf_iterator<char>(in)),
                         std::istreambuf_iterator<char>());

  std::regex device_re(R"(device\s*:\s*([^\s#]+))");
  std::smatch m;
  if (std::regex_search(yaml, m, device_re) && m.size() > 1) {
    out.device = m[1].str();
  }

  out.width = parseUint(yaml, "capture_width", out.width);
  if (out.width == 0) {
    out.width = parseUint(yaml, "image_width", 1280);
  }
  out.height = parseUint(yaml, "capture_height", out.height);
  if (out.height == 0) {
    out.height = parseUint(yaml, "image_height", 1024);
  }
  out.fps = parseUint(yaml, "framerate", 60);
  out.output_width = parseUint(yaml, "output_width", 640);
  out.output_height = parseUint(yaml, "output_height", 512);

  out.uvc.mode = parseString(yaml, "uvc_control_mode", out.uvc.mode);
  out.uvc.brightness = parseInt(yaml, "brightness", out.uvc.brightness);
  out.uvc.contrast = parseInt(yaml, "contrast", out.uvc.contrast);
  out.uvc.saturation = parseInt(yaml, "saturation", out.uvc.saturation);
  out.uvc.sharpness = parseInt(yaml, "sharpness", out.uvc.sharpness);
  out.uvc.gain = parseInt(yaml, "gain", out.uvc.gain);
  out.uvc.gamma = parseInt(yaml, "gamma", out.uvc.gamma);
  out.uvc.backlight_compensation =
      parseInt(yaml, "backlight_compensation", out.uvc.backlight_compensation);
  out.uvc.exposure = parseInt(yaml, "exposure", out.uvc.exposure);
  out.uvc.white_balance = parseInt(yaml, "white_balance", out.uvc.white_balance);

  out.uvc.brightness_pct = parseInt(yaml, "brightness_pct", out.uvc.brightness_pct);
  out.uvc.contrast_pct = parseInt(yaml, "contrast_pct", out.uvc.contrast_pct);
  out.uvc.saturation_pct = parseInt(yaml, "saturation_pct", out.uvc.saturation_pct);
  out.uvc.sharpness_pct = parseInt(yaml, "sharpness_pct", out.uvc.sharpness_pct);
  out.uvc.gain_pct = parseInt(yaml, "gain_pct", out.uvc.gain_pct);
  out.uvc.gamma_pct = parseInt(yaml, "gamma_pct", out.uvc.gamma_pct);
  out.uvc.backlight_compensation_pct = parseInt(
      yaml, "backlight_compensation_pct", out.uvc.backlight_compensation_pct);
  out.uvc.exposure_pct = parseInt(yaml, "exposure_pct", out.uvc.exposure_pct);
  out.uvc.white_balance_pct =
      parseInt(yaml, "white_balance_pct", out.uvc.white_balance_pct);

  out.uvc.autoexposure = parseBool(yaml, "autoexposure", out.uvc.autoexposure);
  out.uvc.auto_white_balance =
      parseBool(yaml, "auto_white_balance", out.uvc.auto_white_balance);
  out.uvc.autofocus = parseBool(yaml, "autofocus", out.uvc.autofocus);
  return true;
}

}  // namespace circle::perception
