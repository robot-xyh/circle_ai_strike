#pragma once

#include <string>
#include <vector>

#include "circle/debug_common/telemetry_schema.hpp"
#include "circle/strike_png/strike_png_node_params.hpp"

namespace circle::debug_common {

/** Tunable PNG parameters (node-level + entry handoff + StrikePngController).
 *  Uses target_strike_png.* names for UI parity with the ROS2 PNG node. */
std::vector<ParamEntry> strikePngTunableParams(
    const circle::strike_png::StrikePngNodeParams& params,
    const char* prefix = "target_strike_png.");

std::string strikePngParamsJson(
    const circle::strike_png::StrikePngNodeParams& params,
    const char* prefix = "target_strike_png.",
    const char* tune_mode = "target_strike_png");

#if defined(CIRCLE_STRIKE_HAS_YAML) && CIRCLE_STRIKE_HAS_YAML
/** Apply one {"name":"...","value":...} update. Accepts target_strike_png.* or
 *  strike_png.* prefixes. */
bool applyStrikePngParamUpdate(circle::strike_png::StrikePngNodeParams& params,
                               const std::string& json);

/** Merge live tunable JSON (/api/params.json body) onto file-backed defaults. */
bool strikePngParamsFromTunableJson(const std::string& yaml_path,
                                    const std::string& tunable_json,
                                    circle::strike_png::StrikePngNodeParams& out,
                                    std::string* error_out = nullptr);

/** Write tunable PNG params into output_yaml_path (template from source). */
bool saveStrikePngTunableParamsToYaml(
    const std::string& source_yaml_path, const std::string& output_yaml_path,
    const circle::strike_png::StrikePngNodeParams& params,
    std::string* error_out = nullptr);
#endif

}  // namespace circle::debug_common
