#pragma once

#include <string>

#include "circle/types/fc_state.hpp"

namespace circle::types {

enum class DeploymentProfile : uint8_t {
  Sim = 0,
  Real = 1,
};

struct DeploymentConfig {
  DeploymentProfile profile{DeploymentProfile::Real};
  FcBackend fc_backend{FcBackend::Px4};
};

inline FcBackend parseFcBackend(const std::string& s) {
  if (s == "betaflight" || s == "bf") {
    return FcBackend::Betaflight;
  }
  return FcBackend::Px4;
}

inline DeploymentProfile parseDeploymentProfile(const std::string& s) {
  if (s == "sim" || s == "simulation") {
    return DeploymentProfile::Sim;
  }
  return DeploymentProfile::Real;
}

}  // namespace circle::types
