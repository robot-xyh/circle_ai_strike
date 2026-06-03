#pragma once

#include <string>

#include "circle/strike/strike_params.hpp"

namespace circle::strike {

#if defined(CIRCLE_STRIKE_HAS_YAML) && CIRCLE_STRIKE_HAS_YAML

/** Load StrikeParams from nested strike YAML. Missing keys keep defaults. */
StrikeParams loadStrikeParamsFromYaml(const std::string& path);

#endif

}  // namespace circle::strike
