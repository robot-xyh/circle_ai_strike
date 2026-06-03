#include "circle/debug_common/telemetry_schema.hpp"

#include <sstream>

#include "circle/debug_common/strike_param_tune.hpp"

namespace circle::debug_common {

std::vector<ParamEntry> defaultStrikeTunableParams() {
  circle::strike::StrikeParams defaults;
  return strikeCoreTunableParams(defaults, "target_strike.");
}

std::string paramsJson(const std::vector<ParamEntry>& params) {
  std::ostringstream oss;
  oss << R"({"params":[)";
  bool first = true;
  for (const auto& p : params) {
    if (!first) {
      oss << ',';
    }
    first = false;
    oss << R"({"name":")" << p.name << R"(","value":)";
    if (p.is_bool) {
      oss << (p.value != 0.0 ? "true" : "false");
    } else {
      oss << p.value;
    }
    oss << R"(,"min":)" << p.min << R"(,"max":)" << p.max
        << R"(,"step":)" << p.step;
    if (p.is_bool) {
      oss << R"(,"type":"bool")";
    }
    oss << '}';
  }
  oss << "]}";
  return oss.str();
}

std::string bfVideoStatusJson(bool telemetry_open, bool preview_open,
                              const std::string& mode) {
  std::ostringstream oss;
  const bool ok = preview_open;
  oss << R"({"ok":)" << (ok ? "true" : "false")
      << R"(,"mode":")" << mode << R"(","telemetry_open":)"
      << (telemetry_open ? "true" : "false")
      << R"(,"preview_open":)" << (preview_open ? "true" : "false")
      << R"(,"transport":"mjpeg","preview_browser":"mjpeg","h264_mode":false)";
  if (preview_open) {
    oss << R"(,"mjpeg_preview_path":"/api/video/mjpeg")";
  }
  oss << R"(,"status":")";
  if (!preview_open) {
    oss << "preview SHM not open";
  } else if (!telemetry_open) {
    oss << "MJPEG preview; telemetry SHM not open";
  } else {
    oss << "MJPEG preview (/api/video/mjpeg)";
  }
  oss << "\"}";
  return oss.str();
}

}  // namespace circle::debug_common
