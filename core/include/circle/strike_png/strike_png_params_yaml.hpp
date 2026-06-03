#pragma once

#include <string>

#include "circle/strike_png/strike_png_node_params.hpp"

namespace circle::strike_png {

#if defined(CIRCLE_STRIKE_HAS_YAML) && CIRCLE_STRIKE_HAS_YAML

/** Load StrikePngNodeParams from the `strike_png:` block (missing keys keep
 *  defaults). Flat key names match the PX4 target_strike_png.* parameters. */
StrikePngNodeParams loadStrikePngParamsFromYaml(const std::string& path);

#endif

}  // namespace circle::strike_png
