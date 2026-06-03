#pragma once

#include "circle/types/fc_state.hpp"

namespace circle::fc_iface {

class IVehicleStateSource {
 public:
  virtual ~IVehicleStateSource() = default;

  virtual circle::types::FcState snapshot() const = 0;
};

}  // namespace circle::fc_iface
