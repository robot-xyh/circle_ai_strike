#pragma once

#include <string>
#include <vector>

#include "circle/debug_common/telemetry_schema.hpp"
#include "circle/strike/strike_params.hpp"

namespace circle::debug_common {

/** Tunable strike-core parameters for Profile C (BF). Uses target_strike.* names for UI parity with ROS2 debug. */
std::vector<ParamEntry> strikeCoreTunableParams(const circle::strike::StrikeParams& params,
                                                const char* prefix = "target_strike.");

std::string strikeCoreParamsJson(const circle::strike::StrikeParams& params,
                                 const char* prefix = "target_strike.",
                                 const char* tune_mode = "target_strike");

#if defined(CIRCLE_STRIKE_HAS_YAML) && CIRCLE_STRIKE_HAS_YAML
/** Apply one {"name":"...","value":...} update. Accepts target_strike.* or strike.* prefixes. */
bool applyStrikeParamUpdate(circle::strike::StrikeParams& params, const std::string& json);

/** Merge live tunable JSON (/api/params.json body) onto file-backed defaults. */
bool strikeParamsFromTunableJson(const std::string& yaml_path,
                                 const std::string& tunable_json,
                                 circle::strike::StrikeParams& out,
                                 std::string* error_out = nullptr);

/** Write tunable strike params into output_yaml_path (template from source_yaml_path). */
bool saveStrikeTunableParamsToYaml(const std::string& source_yaml_path,
                                   const std::string& output_yaml_path,
                                   const circle::strike::StrikeParams& params,
                                   std::string* error_out = nullptr);
#endif

}  // namespace circle::debug_common
