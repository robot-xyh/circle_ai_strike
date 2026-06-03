#pragma once

#include "circle/types/rate_command.hpp"

namespace circle::fc_iface {

class IRateSink {
 public:
  virtual ~IRateSink() = default;

  virtual void publishRates(const circle::types::RateCommand& command,
                            const circle::types::SafetyContext& safety) = 0;

  virtual void resetActivationLatch() = 0;
};

}  // namespace circle::fc_iface
