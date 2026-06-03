#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace circle::debug_common {

struct SeriesPoint {
  double t{0.0};
  double v{0.0};
};

struct ParamEntry {
  std::string name;
  bool is_bool{false};
  double value{0.0};
  double min{0.0};
  double max{1.0};
  double step{0.01};
};

std::vector<ParamEntry> defaultStrikeTunableParams();
std::string paramsJson(const std::vector<ParamEntry>& params);
std::string bfVideoStatusJson(bool telemetry_open, bool preview_open,
                              const std::string& mode);

}  // namespace circle::debug_common
