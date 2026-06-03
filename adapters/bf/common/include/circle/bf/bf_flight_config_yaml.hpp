#pragma once

#include <cstdint>
#include <string>

#include "circle/bf/bf_rc_mapper.hpp"

#if defined(CIRCLE_STRIKE_HAS_YAML) && CIRCLE_STRIKE_HAS_YAML
#include <yaml-cpp/yaml.h>
#endif

namespace circle::bf {

/** BF flight MSP link + OVERRIDE / passthrough settings (`bf_flight.msp` in YAML). */
struct BfFlightMspConfig {
  std::string device{"/dev/ttyACM0"};
  int baud{115200};
  bool passthrough_in_dry_run{true};
  bool passthrough_log{true};
  double passthrough_log_interval_s{1.0};
  uint16_t passthrough_throttle_jump_pwm{40};
  uint32_t override_mode_flag{0x2000000U};
  bool override_mode_flag_auto{true};
  uint32_t override_channels_mask{0x0FU};
  uint16_t passthrough_channel_count{16};
  double passthrough_hz{50.0};
  double live_publish_hz{0.0};
  double override_grace_hold_s{0.35};
  // Scheme B MSP budget: single I/O thread fires SET_RAW_RC every cycle (top
  // priority), and interleaves queries by these divisors of the I/O cycle rate.
  // ATTITUDE every Nth cycle (default /2 ≈ 50 Hz), STATUS every Mth (default
  // /7 ≈ 14 Hz). Missing replies keep the last cached value (see *_age_ms).
  uint32_t attitude_poll_divisor{2};
  uint32_t status_poll_divisor{7};
};

struct BfFlightYamlSections {
  BfFlightMspConfig msp;
  BfRcMapperConfig rc;
};

#if defined(CIRCLE_STRIKE_HAS_YAML) && CIRCLE_STRIKE_HAS_YAML
void loadBfFlightMspFromYaml(const YAML::Node& msp, BfFlightMspConfig& cfg);
void loadBfFlightRcFromYaml(const YAML::Node& rc, BfRcMapperConfig& cfg);
BfFlightYamlSections loadBfFlightYamlSectionsFromFile(const std::string& path);
#endif

}  // namespace circle::bf
