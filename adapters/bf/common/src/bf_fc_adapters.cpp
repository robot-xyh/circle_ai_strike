#include "circle/bf/bf_fc_adapters.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

#include "circle/bf/logger.hpp"
#include "circle/types/time.hpp"

namespace circle::bf {

namespace {

std::vector<uint16_t> normalizeRcChannels(const std::vector<uint16_t>& in,
                                          const BfRcMapperConfig& cfg,
                                          uint16_t passthrough_channel_count) {
  const size_t count =
      std::max<size_t>({cfg.channel_count, passthrough_channel_count, in.size()});
  const size_t capped = std::min(count, static_cast<size_t>(16));
  std::vector<uint16_t> out(capped, cfg.rc_mid);
  const size_t n = std::min(in.size(), out.size());
  for (size_t i = 0; i < n; ++i) {
    out[i] = clampRcPwm(in[i]);
  }
  return out;
}

void mergeMaskedChannels(std::vector<uint16_t>* sent,
                         const std::vector<uint16_t>& latched,
                         uint32_t mask) {
  if (!sent) {
    return;
  }
  const size_t n = std::min(sent->size(), latched.size());
  for (size_t i = 0; i < n; ++i) {
    if ((mask & (1U << i)) != 0U) {
      (*sent)[i] = latched[i];
    }
  }
}

/** OVERRIDE: latched physical sticks; merged MSP_RC is corrupt for ch0-3. */
void buildOverridePassthroughSend(std::vector<uint16_t>* sent,
                                  const std::vector<uint16_t>& latched,
                                  const std::vector<uint16_t>& live,
                                  uint32_t mask,
                                  uint16_t aux_start_channel) {
  if (!sent || latched.empty()) {
    return;
  }
  *sent = latched;
  const size_t n = std::min(sent->size(), live.size());
  for (size_t i = 0; i < n; ++i) {
    if ((mask & (1U << i)) != 0U) {
      continue;
    }
    if (i >= aux_start_channel) {
      (*sent)[i] = live[i];
    }
  }
}

uint16_t channelAt(const std::vector<uint16_t>& channels, uint16_t index,
                   uint16_t fallback) {
  return index < channels.size() ? channels[index] : fallback;
}

/** BF SET_RAW_RC wire order is AETR (T@2 Y@3); MSP_RC returns rcData map (here RPYT). */
std::vector<uint16_t> toMspSetRawRcWireOrder(
    const std::vector<uint16_t>& rc_data_order, const BfRcMapperConfig& cfg) {
  std::vector<uint16_t> wire = rc_data_order;
  if (wire.size() < 4) {
    wire.resize(4, cfg.rc_mid);
  }
  wire[0] = clampRcPwm(channelAt(rc_data_order, cfg.roll_channel, cfg.rc_mid));
  wire[1] = clampRcPwm(channelAt(rc_data_order, cfg.pitch_channel, cfg.rc_mid));
  wire[2] =
      clampRcPwm(channelAt(rc_data_order, cfg.throttle_channel, cfg.rc_min));
  wire[3] = clampRcPwm(channelAt(rc_data_order, cfg.yaw_channel, cfg.rc_mid));
  return wire;
}

}  // namespace

BfRateSink::BfRateSink(std::shared_ptr<MspClient> client, BfRcMapper mapper,
                       MspPassthroughDebugConfig passthrough_debug)
    : client_(std::move(client)),
      mapper_(std::move(mapper)),
      passthrough_debug_(passthrough_debug) {}

void BfRateSink::recordSentWire(const std::vector<uint16_t>& wire) {
  const auto& cfg = mapper_.config();
  const float span = static_cast<float>(cfg.rc_max - cfg.rc_min);
  const float half_span = span * 0.5F;
  if (wire.size() >= 4) {
    last_sent_roll_pwm_ = wire[0];
    last_sent_pitch_pwm_ = wire[1];
    last_sent_throttle_pwm_ = wire[2];
    last_sent_yaw_pwm_ = wire[3];
    if (half_span > 0.0F) {
      last_sent_roll_norm_ =
          (static_cast<float>(last_sent_roll_pwm_) -
           static_cast<float>(cfg.rc_mid)) /
          half_span;
      last_sent_pitch_norm_ =
          (static_cast<float>(last_sent_pitch_pwm_) -
           static_cast<float>(cfg.rc_mid)) /
          half_span;
      last_sent_throttle_norm_ =
          (static_cast<float>(last_sent_throttle_pwm_) -
           static_cast<float>(cfg.rc_min)) /
          span;
    }
    last_sent_valid_ = true;
  } else if (wire.size() >= 3) {
    last_sent_throttle_pwm_ = wire[2];
    if (span > 0.0F) {
      last_sent_throttle_norm_ =
          (static_cast<float>(last_sent_throttle_pwm_) -
           static_cast<float>(cfg.rc_min)) /
          span;
    }
    last_sent_valid_ = true;
  }
}

BfRateSink::LastSentWire BfRateSink::lastSentWire() const {
  std::lock_guard<std::mutex> lk(mu_);
  LastSentWire out;
  out.roll_pwm = last_sent_roll_pwm_;
  out.pitch_pwm = last_sent_pitch_pwm_;
  out.yaw_pwm = last_sent_yaw_pwm_;
  out.throttle_pwm = last_sent_throttle_pwm_;
  out.roll_norm = last_sent_roll_norm_;
  out.pitch_norm = last_sent_pitch_norm_;
  out.valid = last_sent_valid_;
  return out;
}

BfRateSink::LastSentThrottle BfRateSink::lastSentThrottle() const {
  std::lock_guard<std::mutex> lk(mu_);
  LastSentThrottle out;
  out.pwm = last_sent_throttle_pwm_;
  out.norm = last_sent_throttle_norm_;
  out.valid = last_sent_valid_;
  return out;
}

bool BfRateSink::mspOverrideActive() const {
  std::lock_guard<std::mutex> lk(mu_);
  return cached_override_active_;
}

bool BfRateSink::mspOverrideRawActive() const {
  std::lock_guard<std::mutex> lk(mu_);
  return cached_override_raw_active_;
}

bool BfRateSink::mspOverrideGraceHoldActive() const {
  std::lock_guard<std::mutex> lk(mu_);
  return override_grace_hold_active_;
}

uint32_t BfRateSink::cachedModeFlags() const {
  std::lock_guard<std::mutex> lk(mu_);
  return cached_status_valid_ ? cached_status_.mode_flags : 0U;
}

uint32_t BfRateSink::overrideModeFlag() const {
  return passthrough_debug_.override_mode_flag;
}

void BfRateSink::refreshMspStatusFromPoll() {
  std::lock_guard<std::mutex> lk(mu_);
  refreshMspContextLocked(true);
}

bool BfRateSink::applyOverrideDebouncing(bool raw_active) {
  // Debounce removed: the grace-hold window already smooths brief raw=0 gaps,
  // so a raw OVERRIDE bit drop goes straight into grace instead of waiting for
  // N consecutive misses (which only added latency + double-smoothing).
  cached_override_raw_active_ = raw_active;

  if (raw_active) {
    override_grace_hold_active_ = false;
    override_debounced_active_ = true;
  } else if (override_debounced_active_) {
    override_debounced_active_ = false;
    if (passthrough_debug_.override_grace_hold_s > 0.0 &&
        override_latch_valid_) {
      override_grace_hold_active_ = true;
      override_grace_until_ =
          std::chrono::steady_clock::now() +
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
              std::chrono::duration<double>(
                  passthrough_debug_.override_grace_hold_s));
    }
  }

  if (override_grace_hold_active_ &&
      std::chrono::steady_clock::now() >= override_grace_until_) {
    override_grace_hold_active_ = false;
  }

  cached_override_active_ =
      override_debounced_active_ || override_grace_hold_active_;
  return cached_override_active_;
}

void BfRateSink::updatePhysicalAndOverrideLatch(
    const std::vector<uint16_t>& normalized, bool effective_override_active) {
  last_rc_read_normalized_ = normalized;

  const bool in_hold_window =
      effective_override_active || override_grace_hold_active_;

  if (!in_hold_window) {
    last_physical_channels_ = normalized;
    override_was_active_ = false;
    override_latch_valid_ = false;
    return;
  }

  const bool override_rising =
      effective_override_active && !override_was_active_;
  override_was_active_ = effective_override_active;

  if (effective_override_active &&
      (override_rising || !override_latch_valid_) &&
      !last_physical_channels_.empty()) {
    override_latched_channels_ = last_physical_channels_;
    override_latch_valid_ = true;
    if (passthrough_debug_.log_enabled && override_rising &&
        logEnabled(LogLevel::Debug)) {
      const auto& cfg = mapper_.config();
      const auto wire = toMspSetRawRcWireOrder(override_latched_channels_, cfg);
      logDebug("bf_flight/msp/override_latch: capture R=",
               channelAt(override_latched_channels_, cfg.roll_channel, 0),
               " P=", channelAt(override_latched_channels_, cfg.pitch_channel, 0),
               " Y=", channelAt(override_latched_channels_, cfg.yaw_channel, 0),
               " T=", channelAt(override_latched_channels_, cfg.throttle_channel, 0),
               " wireT=", (wire.size() >= 3 ? wire[2] : 0),
               " (mask=0x", std::hex,
               passthrough_debug_.override_channels_mask, std::dec, ")");
    }
  }
}

void BfRateSink::applyPollContextLocked(const MspStatus& status, bool status_ok,
                                        const std::vector<uint16_t>& rc_channels,
                                        bool rc_ok) {
  last_msp_context_at_ = std::chrono::steady_clock::now();

  bool raw_override_active = cached_override_raw_active_;
  if (status_ok) {
    cached_status_valid_ = true;
    cached_status_ = status;
    raw_override_active =
        (status.mode_flags & passthrough_debug_.override_mode_flag) != 0U;
    applyOverrideDebouncing(raw_override_active);
  } else {
    cached_status_valid_ = false;
    if (override_grace_hold_active_ &&
        std::chrono::steady_clock::now() >= override_grace_until_) {
      override_grace_hold_active_ = false;
    }
    cached_override_active_ =
        override_debounced_active_ || override_grace_hold_active_;
  }

  if (rc_ok && !rc_channels.empty()) {
    const auto normalized = normalizeRcChannels(
        rc_channels, mapper_.config(),
        passthrough_debug_.passthrough_channel_count);
    updatePhysicalAndOverrideLatch(normalized, cached_override_active_);
  }
}

void BfRateSink::refreshMspContextLocked(bool force_refresh) {
  const auto now = std::chrono::steady_clock::now();
  if (!force_refresh && last_msp_context_at_.time_since_epoch().count() != 0 &&
      now - last_msp_context_at_ < std::chrono::milliseconds(100)) {
    return;
  }

  MspStatus status{};
  const bool status_ok = client_->readStatus(status);
  std::vector<uint16_t> read_channels;
  const bool rc_ok =
      client_->readRc(&read_channels) && !read_channels.empty();
  applyPollContextLocked(status, status_ok, read_channels, rc_ok);
}

void BfRateSink::ingestPoll(const MspPollResult& poll) {
  std::lock_guard<std::mutex> lk(mu_);
  applyPollContextLocked(poll.status, poll.status_ok, poll.rc, poll.rc_ok);
}

bool BfRateSink::needsRcPoll() const {
  std::lock_guard<std::mutex> lk(mu_);
  // Once OVERRIDE is active and the physical sticks are latched, BF's MSP_RC for
  // masked channels is latched/garbage and unused — stop polling it.
  return !(cached_override_active_ && override_latch_valid_);
}

BfRateSink::LastSentThrottle BfRateSink::lastLatchedPhysicalThrottle() const {
  std::lock_guard<std::mutex> lk(mu_);
  LastSentThrottle out;
  if (!override_latch_valid_ || override_latched_channels_.empty()) {
    return out;
  }
  const auto& cfg = mapper_.config();
  if (cfg.throttle_channel >= override_latched_channels_.size()) {
    return out;
  }
  out.pwm = override_latched_channels_[cfg.throttle_channel];
  // Guard against a corrupted latch: BF returns latched/garbage RC for masked
  // channels right at the OVERRIDE switch instant. If the captured throttle PWM
  // is outside the configured stick range (with a small tolerance), treat the
  // latch as invalid so callers fall back to a safe value (hover) instead of a
  // bogus near-zero throttle.
  constexpr uint16_t kPwmTol = 50;
  const uint16_t lo = cfg.rc_min > kPwmTol ? cfg.rc_min - kPwmTol : 0;
  const uint16_t hi = cfg.rc_max + kPwmTol;
  if (out.pwm < lo || out.pwm > hi) {
    return LastSentThrottle{};
  }
  const float span = static_cast<float>(cfg.rc_max - cfg.rc_min);
  if (span > 0.0F) {
    out.norm = (static_cast<float>(out.pwm) - static_cast<float>(cfg.rc_min)) /
               span;
  }
  out.valid = true;
  return out;
}

void BfRateSink::logLiveSkip(const char* reason,
                             const circle::types::RateCommand& command,
                             bool override_active, bool armed_ok,
                             bool override_latch_active,
                             bool algo_rates_active) {
  if (!passthrough_debug_.log_enabled) {
    return;
  }
  const bool rates_nonzero =
      std::fabs(command.roll_rate_rad_s) > 1.0e-4F ||
      std::fabs(command.pitch_rate_rad_s) > 1.0e-4F ||
      std::fabs(command.yaw_rate_rad_s) > 1.0e-4F;
  if (!rates_nonzero && !algo_rates_active) {
    return;
  }
  const auto now = std::chrono::steady_clock::now();
  static std::chrono::steady_clock::time_point last_skip_log{};
  if (last_skip_log.time_since_epoch().count() != 0 &&
      now - last_skip_log <
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
              std::chrono::duration<double>(
                  passthrough_debug_.log_interval_s))) {
    return;
  }
  last_skip_log = now;
  circle::bf::logInfo(
      "bf_flight/msp/live_skip: reason=", reason, " armed=", (armed_ok ? 1 : 0),
      " override=", (override_active ? 1 : 0),
      " latch=", (override_latch_active ? 1 : 0),
      " roll_sp=", command.roll_rate_rad_s,
      " pitch_sp=", command.pitch_rate_rad_s,
      " yaw_sp=", command.yaw_rate_rad_s,
      " thrust_z=", command.thrust_z,
      " (telemetry rates not on wire; need armed+OVERRIDE+algo)");
}

void BfRateSink::logLivePublish(const std::vector<uint16_t>& wire,
                                bool override_active, bool send_ok,
                                bool passthrough_hold,
                                const circle::types::RateCommand* command,
                                const std::vector<uint16_t>* logical_channels,
                                bool armed_ok, bool override_latch_active,
                                bool algo_commanding, bool algo_rates_active,
                                bool algo_throttle_active) {
  if (!passthrough_debug_.log_enabled) {
    return;
  }
  const auto now = std::chrono::steady_clock::now();
  const bool rates_nonzero =
      command != nullptr &&
      (std::fabs(command->roll_rate_rad_s) > 1.0e-4F ||
       std::fabs(command->pitch_rate_rad_s) > 1.0e-4F ||
       std::fabs(command->yaw_rate_rad_s) > 1.0e-4F);
  const bool periodic =
      last_log_at_.time_since_epoch().count() == 0 ||
      now - last_log_at_ >=
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
              std::chrono::duration<double>(passthrough_debug_.log_interval_s));
  if (!periodic && send_ok && !rates_nonzero && passthrough_hold) {
    return;
  }
  last_log_at_ = now;

  std::ostringstream oss;
  oss << "bf_flight/msp/live:"
      << " armed=" << (cached_status_valid_ && cached_status_.armed ? 1 : 0)
      << " armed_ok=" << (armed_ok ? 1 : 0)
      << " override=" << (override_active ? 1 : 0)
      << " override_raw=" << (cached_override_raw_active_ ? 1 : 0)
      << " override_grace=" << (override_grace_hold_active_ ? 1 : 0)
      << " latch=" << (override_latch_active ? 1 : 0)
      << " algo_cmd=" << (algo_commanding ? 1 : 0)
      << " algo_rates=" << (algo_rates_active ? 1 : 0)
      << " algo_thr=" << (algo_throttle_active ? 1 : 0)
      << " passthrough_hold=" << (passthrough_hold ? 1 : 0)
      << " send=" << (send_ok ? "ok" : "fail")
      << " ok=" << live_publish_ok_ << " fail=" << live_publish_fail_;
  if (command != nullptr) {
    oss << " roll_sp=" << command->roll_rate_rad_s
        << " pitch_sp=" << command->pitch_rate_rad_s
        << " yaw_sp=" << command->yaw_rate_rad_s
        << " thrust_z=" << command->thrust_z;
  }
  if (logical_channels != nullptr && !logical_channels->empty()) {
    const auto& cfg = mapper_.config();
    oss << " chR=" << channelAt(*logical_channels, cfg.roll_channel, 0)
        << " chP=" << channelAt(*logical_channels, cfg.pitch_channel, 0)
        << " chT=" << channelAt(*logical_channels, cfg.throttle_channel, 0)
        << " chY=" << channelAt(*logical_channels, cfg.yaw_channel, 0);
  }
  if (wire.size() >= 4) {
    oss << " wireR=" << wire[0] << " wireP=" << wire[1]
        << " wireT=" << wire[2] << " wireY=" << wire[3];
  }
  if (!send_ok && !client_->lastError().empty()) {
    oss << " err=" << client_->lastError();
  }
  if (!override_active && !live_override_warned_) {
    oss << " **MSP_OVERRIDE_inactive** (BF ignores SET_RAW_RC for R/P/T)";
    live_override_warned_ = true;
  }
  if (rates_nonzero && passthrough_hold) {
    oss << " **RATES_BLOCKED_BY_PASSTHROUGH_HOLD**";
  }
  if (rates_nonzero && !armed_ok) {
    oss << " **RATES_NOT_SENT_DISARMED**";
  }
  circle::bf::logInfo(oss.str());
}

bool BfRateSink::computeWire(const circle::types::RateCommand& command,
                             const circle::types::SafetyContext& safety,
                             StagedSend& out) {
  // Caller holds mu_. No serial I/O, no rate gate: state comes from the cached
  // poll context that the single MSP I/O thread keeps fresh via ingestPoll().
  const bool override_active = cached_override_active_;
  const bool armed_ok = !safety.require_armed_to_command || safety.armed;
  const bool override_latch_active = override_active && override_latch_valid_;

  constexpr float kAlgoThrottleEpsilon = 1.0e-4F;
  constexpr float kAlgoRateEpsilon = 1.0e-4F;
  const bool algo_throttle_active = command.thrust_z > kAlgoThrottleEpsilon;
  const bool algo_rates_active =
      std::fabs(command.roll_rate_rad_s) > kAlgoRateEpsilon ||
      std::fabs(command.pitch_rate_rad_s) > kAlgoRateEpsilon ||
      std::fabs(command.yaw_rate_rad_s) > kAlgoRateEpsilon;
  const bool algo_commanding =
      armed_ok && (algo_throttle_active || algo_rates_active);

  if (!override_active) {
    logLiveSkip("override_inactive", command, override_active, armed_ok,
                override_latch_active, algo_rates_active);
    return false;
  }
  if (!armed_ok && !override_latch_active) {
    logLiveSkip("disarmed_no_override_latch", command, override_active, armed_ok,
                override_latch_active, algo_rates_active);
    return false;
  }

  const auto& cfg = mapper_.config();
  std::vector<uint16_t> sent_channels;

  if (override_latch_active && !algo_commanding) {
    // Same path as dry_run passthrough: latched physical R/P/Y/T + live aux.
    sent_channels = override_latched_channels_;
    const auto& live = last_rc_read_normalized_.empty()
                           ? last_physical_channels_
                           : last_rc_read_normalized_;
    buildOverridePassthroughSend(
        &sent_channels, override_latched_channels_, live,
        passthrough_debug_.override_channels_mask, cfg.aux_arm_channel);
  } else {
    sent_channels = mapper_.mapRates(command, safety.armed);
    sent_channels.resize(
        std::max<size_t>(sent_channels.size(),
                         passthrough_debug_.passthrough_channel_count),
        cfg.rc_mid);

    const uint32_t mask = passthrough_debug_.override_channels_mask;
    const auto& live = last_rc_read_normalized_.empty()
                           ? last_physical_channels_
                           : last_rc_read_normalized_;
    for (size_t i = 0; i < sent_channels.size(); ++i) {
      if ((mask & (1U << i)) == 0U && i < live.size()) {
        sent_channels[i] = live[i];
      }
    }

    if (override_latch_active && !algo_throttle_active &&
        cfg.throttle_channel < override_latched_channels_.size() &&
        cfg.throttle_channel < sent_channels.size()) {
      sent_channels[cfg.throttle_channel] =
          override_latched_channels_[cfg.throttle_channel];
    }
    if (override_latch_active && !algo_rates_active) {
      if (cfg.roll_channel < override_latched_channels_.size() &&
          cfg.roll_channel < sent_channels.size()) {
        sent_channels[cfg.roll_channel] =
            override_latched_channels_[cfg.roll_channel];
      }
      if (cfg.pitch_channel < override_latched_channels_.size() &&
          cfg.pitch_channel < sent_channels.size()) {
        sent_channels[cfg.pitch_channel] =
            override_latched_channels_[cfg.pitch_channel];
      }
      if (cfg.yaw_channel < override_latched_channels_.size() &&
          cfg.yaw_channel < sent_channels.size()) {
        sent_channels[cfg.yaw_channel] =
            override_latched_channels_[cfg.yaw_channel];
      }
    }
  }

  out.wire = toMspSetRawRcWireOrder(sent_channels, cfg);
  out.sent_channels = std::move(sent_channels);
  out.command = command;
  out.has_command = true;
  out.override_active = override_active;
  out.armed_ok = armed_ok;
  out.override_latch_active = override_latch_active;
  out.algo_commanding = algo_commanding;
  out.algo_rates_active = algo_rates_active;
  out.algo_throttle_active = algo_throttle_active;
  out.passthrough_hold = override_latch_active && !algo_commanding;
  out.valid = true;
  return true;
}

bool BfRateSink::computePhysicalHold(StagedSend& out) {
  // Caller holds mu_. No serial I/O, no rate gate.
  const bool override_active = cached_override_active_;
  if (!override_active || !override_latch_valid_) {
    return false;
  }
  const auto& cfg = mapper_.config();
  std::vector<uint16_t> sent_channels = override_latched_channels_;
  const auto& live = last_rc_read_normalized_.empty()
                         ? last_physical_channels_
                         : last_rc_read_normalized_;
  buildOverridePassthroughSend(
      &sent_channels, override_latched_channels_, live,
      passthrough_debug_.override_channels_mask, cfg.aux_arm_channel);

  out.wire = toMspSetRawRcWireOrder(sent_channels, cfg);
  out.sent_channels = std::move(sent_channels);
  out.has_command = false;
  out.override_active = override_active;
  out.armed_ok = false;
  out.override_latch_active = override_latch_valid_;
  out.algo_commanding = false;
  out.algo_rates_active = false;
  out.algo_throttle_active = false;
  out.passthrough_hold = true;
  out.valid = true;
  return true;
}

void BfRateSink::stageWire(const circle::types::RateCommand& command,
                           const circle::types::SafetyContext& safety) {
  if (safety.dry_run) {
    return;
  }
  std::lock_guard<std::mutex> lk(mu_);
  StagedSend s;
  if (computeWire(command, safety, s)) {
    staged_ = std::move(s);
    ++staged_seq_;
    ++staged_publish_count_;
  } else {
    staged_.valid = false;
  }
}

void BfRateSink::stagePhysicalHold() {
  std::lock_guard<std::mutex> lk(mu_);
  StagedSend s;
  if (computePhysicalHold(s)) {
    staged_ = std::move(s);
    ++staged_seq_;
    ++staged_publish_count_;
  } else {
    staged_.valid = false;
  }
}

bool BfRateSink::flushStagedWire() {
  std::lock_guard<std::mutex> lk(mu_);
  if (!staged_.valid) {
    return false;
  }
  const bool send_ok = client_->writeRawRc(staged_.wire);
  if (send_ok) {
    ++live_publish_ok_;
    recordSentWire(staged_.wire);
  } else {
    ++live_publish_fail_;
  }
  logLivePublish(staged_.wire, staged_.override_active, send_ok,
                 staged_.passthrough_hold,
                 staged_.has_command ? &staged_.command : nullptr,
                 &staged_.sent_channels, staged_.armed_ok,
                 staged_.override_latch_active, staged_.algo_commanding,
                 staged_.algo_rates_active, staged_.algo_throttle_active);
  flushed_seq_ = staged_seq_;
  return send_ok;
}

void BfRateSink::publishRates(const circle::types::RateCommand& command,
                              const circle::types::SafetyContext& safety) {
  // Scheme B: the control loop no longer writes serial directly. publishRates
  // just stages the wire; the MSP I/O thread flushes it (SET first each cycle).
  stageWire(command, safety);
}

void BfRateSink::publishOverridePhysicalHold() { stagePhysicalHold(); }

uint64_t BfRateSink::wirePublishCount() const {
  std::lock_guard<std::mutex> lk(mu_);
  return staged_publish_count_;
}

void BfRateSink::logPassthrough(const MspStatus& status,
                                const std::vector<uint16_t>& read_channels,
                                const std::vector<uint16_t>& sent_channels,
                                bool override_latch_active,
                                bool force) const {
  if (!passthrough_debug_.log_enabled || !logEnabled(LogLevel::Debug)) {
    return;
  }
  const auto& cfg = mapper_.config();
  const uint16_t thr_idx = cfg.throttle_channel;
  const uint16_t thr_read = channelAt(read_channels, thr_idx, 0);
  const uint16_t thr_wire =
      sent_channels.size() >= 3 ? sent_channels[2] : cfg.rc_min;
  const int thr_delta =
      override_latch_active
          ? static_cast<int>(thr_wire) -
                static_cast<int>(last_throttle_read_)
          : static_cast<int>(thr_read) -
                static_cast<int>(last_throttle_read_);

  const auto now = std::chrono::steady_clock::now();
  const bool mode_changed = status.mode_flags != last_mode_flags_;
  const bool throttle_jump =
      last_throttle_read_ != 0 &&
      std::abs(thr_delta) >= static_cast<int>(passthrough_debug_.throttle_jump_pwm);
  const bool periodic =
      last_log_at_.time_since_epoch().count() == 0 ||
      now - last_log_at_ >=
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
              std::chrono::duration<double>(passthrough_debug_.log_interval_s));

  if (!force && !mode_changed && !throttle_jump && !periodic) {
    return;
  }

  std::ostringstream oss;
  oss << "bf_flight/msp/passthrough:"
      << " armed=" << (status.armed ? 1 : 0)
      << " mode=0x" << std::hex << status.mode_flags << std::dec
      << " nread=" << read_channels.size()
      << " nsent=" << sent_channels.size();
  if (mode_changed && last_mode_flags_ != 0) {
    oss << " mode_chg=0x" << std::hex << last_mode_flags_ << "->0x"
        << status.mode_flags << std::dec;
  }
  if (override_latch_active) {
    oss << " override_latch=1 mask=0x" << std::hex
        << passthrough_debug_.override_channels_mask << std::dec;
  }
  oss << " map=R" << cfg.roll_channel << "/P" << cfg.pitch_channel << "/T"
      << cfg.throttle_channel << "/Y" << cfg.yaw_channel << "/A"
      << cfg.aux_arm_channel;
  oss << " R=" << channelAt(read_channels, cfg.roll_channel, 0)
      << " P=" << channelAt(read_channels, cfg.pitch_channel, 0)
      << " T=" << channelAt(read_channels, cfg.throttle_channel, 0)
      << " Y=" << channelAt(read_channels, cfg.yaw_channel, 0)
      << " A=" << channelAt(read_channels, cfg.aux_arm_channel, 0);
  if (sent_channels.size() >= 4) {
    oss << " wireT=" << sent_channels[2] << " wireY=" << sent_channels[3];
  }
  oss << " read=[";
  for (size_t i = 0; i < read_channels.size(); ++i) {
    if (i > 0) {
      oss << ' ';
    }
    oss << "ch" << i << ':' << read_channels[i];
  }
  oss << "] sent=[";
  for (size_t i = 0; i < sent_channels.size(); ++i) {
    if (i > 0) {
      oss << ' ';
    }
    oss << "ch" << i << ':' << sent_channels[i];
  }
  oss << "]";
  const uint16_t thr_sent_wire =
      sent_channels.size() >= 3 ? sent_channels[2] : cfg.rc_min;
  oss << " Tread=" << thr_read << " Twire=" << thr_sent_wire;
  if (last_throttle_read_ != 0) {
    oss << " Tdelta=" << thr_delta;
  }
  if (throttle_jump) {
    oss << " **THROTTLE_JUMP**";
  }
  logDebug(oss.str());

  last_mode_flags_ = status.mode_flags;
  last_throttle_read_ =
      override_latch_active ? thr_sent_wire : thr_read;
  last_log_at_ = now;
}

bool BfRateSink::publishRcPassthrough() {
  std::lock_guard<std::mutex> lk(mu_);
  last_passthrough_error_.clear();

  MspStatus status{};
  const bool status_ok = client_->readStatus(status);

  std::vector<uint16_t> read_channels;
  if (!client_->readRc(&read_channels) || read_channels.empty()) {
    last_passthrough_error_ = client_->lastError().empty()
                                ? "MSP_RC read failed"
                                : client_->lastError();
    if (passthrough_debug_.log_enabled) {
      logWarn("bf_flight/msp/passthrough: ", last_passthrough_error_);
    }
    return false;
  }

  const auto normalized = normalizeRcChannels(
      read_channels, mapper_.config(),
      passthrough_debug_.passthrough_channel_count);
  const bool raw_override_active =
      status_ok &&
      ((status.mode_flags & passthrough_debug_.override_mode_flag) != 0U);
  applyOverrideDebouncing(raw_override_active);
  const bool override_active = cached_override_active_;
  const bool override_rising = override_active && !override_was_active_;
  updatePhysicalAndOverrideLatch(normalized, override_active);

  std::vector<uint16_t> sent_channels = normalized;
  const bool override_latch_active = override_active && override_latch_valid_;
  if (override_latch_active) {
    buildOverridePassthroughSend(
        &sent_channels, override_latched_channels_, normalized,
        passthrough_debug_.override_channels_mask,
        mapper_.config().aux_arm_channel);
  }

  const auto wire_sent =
      toMspSetRawRcWireOrder(sent_channels, mapper_.config());
  if (!client_->sendRawRc(wire_sent)) {
    last_passthrough_error_ = client_->lastError().empty()
                                ? "MSP_SET_RAW_RC failed"
                                : client_->lastError();
    if (passthrough_debug_.log_enabled) {
      logWarn("bf_flight/msp/passthrough: ", last_passthrough_error_);
    }
    return false;
  }
  recordSentWire(wire_sent);

  if (passthrough_debug_.log_enabled && logEnabled(LogLevel::Debug)) {
    const auto& cfg = mapper_.config();
    const uint16_t thr_idx = cfg.throttle_channel;
    const uint16_t thr_compare = override_latch_active
                                     ? channelAt(sent_channels, thr_idx, 0)
                                     : channelAt(read_channels, thr_idx, 0);
    const int thr_delta =
        static_cast<int>(thr_compare) -
        static_cast<int>(last_throttle_read_);
    const auto now = std::chrono::steady_clock::now();
    const bool mode_changed =
        status_ok && status.mode_flags != last_mode_flags_;
    const bool throttle_jump =
        last_throttle_read_ != 0 &&
        std::abs(thr_delta) >=
            static_cast<int>(passthrough_debug_.throttle_jump_pwm);
    const bool periodic =
        last_log_at_.time_since_epoch().count() == 0 ||
        now - last_log_at_ >=
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(
                    passthrough_debug_.log_interval_s));
    if (periodic || throttle_jump || mode_changed || override_rising) {
      if (status_ok) {
        const auto wire_sent =
            toMspSetRawRcWireOrder(sent_channels, mapper_.config());
        logPassthrough(status, read_channels, wire_sent,
                       override_latch_active, false);
      } else {
        logWarn("bf_flight/msp/passthrough: MSP_STATUS for log failed: ",
                client_->lastError());
      }
    }
  }

  return true;
}

BfStateSource::BfStateSource(std::shared_ptr<MspClient> client,
                             BfRcMapperConfig rc_config,
                             uint32_t override_mode_flag,
                             uint32_t override_channels_mask)
    : client_(std::move(client)),
      rc_config_(rc_config),
      override_mode_flag_(override_mode_flag),
      override_channels_mask_(override_channels_mask) {
  sanitizeRcMapperConfig(rc_config_);
}

float BfStateSource::normalizeThrottlePwm(float pwm,
                                          const BfRcMapperConfig& cfg) {
  const float clamped =
      static_cast<float>(clampRcPwm(static_cast<uint16_t>(std::lround(pwm))));
  const float span = static_cast<float>(cfg.rc_max - cfg.rc_min);
  if (span <= 0.0F) {
    return 0.0F;
  }
  return std::clamp((clamped - static_cast<float>(cfg.rc_min)) / span, 0.0F,
                    1.0F);
}

bool BfStateSource::mspOverrideActive() const {
  std::lock_guard<std::mutex> lk(mu_);
  return override_active_;
}

MspPollStats BfStateSource::pollStats() const {
  std::lock_guard<std::mutex> lk(mu_);
  MspPollStats out;
  out.ok = poll_ok_;
  out.fail = poll_fail_;
  out.last_status_ok = last_poll_status_ok_;
  out.last_attitude_ok = last_poll_attitude_ok_;
  out.last_rc_ok = last_poll_rc_ok_;
  out.last_error = last_poll_error_;
  out.mode_flags = last_mode_flags_;
  out.override_mode_flag = override_mode_flag_;
  out.override_active = override_active_;
  const auto now = std::chrono::steady_clock::now();
  const auto ageMs =
      [&](std::chrono::steady_clock::time_point t) -> double {
    if (t.time_since_epoch().count() == 0) {
      return -1.0;
    }
    return std::chrono::duration<double, std::milli>(now - t).count();
  };
  out.status_age_ms = ageMs(last_status_ok_at_);
  out.attitude_age_ms = ageMs(last_attitude_ok_at_);
  out.rc_age_ms = ageMs(last_rc_ok_at_);
  return out;
}

circle::types::FcState BfStateSource::ingestPoll(
    const MspPollResult& poll) const {
  std::lock_guard<std::mutex> lk(mu_);
  const auto now = std::chrono::steady_clock::now();

  // Merge into the persistent state: only frame types that actually arrived
  // this cycle update their slice; missing frames keep the last good value so
  // an alternating poll schedule never zeroes attitude / clears override.
  circle::types::FcState& out = last_fc_state_;
  out.stamp_ns = circle::types::monotonicNowNs();

  const bool status_ok = poll.status_ok;
  if (status_ok) {
    out.armed = poll.status.armed;
    out.valid = true;
    override_active_ = (poll.status.mode_flags & override_mode_flag_) != 0U;
    last_mode_flags_ = poll.status.mode_flags;
    last_status_ok_at_ = now;
  }
  const bool override_active = override_active_;

  const bool attitude_ok = poll.attitude_ok;
  if (attitude_ok) {
    out.roll_rad = static_cast<float>(poll.attitude.roll_deci_deg) * 0.1F *
                   static_cast<float>(M_PI) / 180.0F;
    out.pitch_rad = static_cast<float>(poll.attitude.pitch_deci_deg) * 0.1F *
                    static_cast<float>(M_PI) / 180.0F;
    out.yaw_rad = static_cast<float>(poll.attitude.yaw_deci_deg) * 0.1F *
                  static_cast<float>(M_PI) / 180.0F;
    out.valid = true;
    last_attitude_ok_at_ = now;
  }

  const bool throttle_masked =
      (override_channels_mask_ & (1U << rc_config_.throttle_channel)) != 0U;
  const bool rc_ok =
      poll.rc_ok && rc_config_.throttle_channel < poll.rc.size();
  if (rc_ok) {
    const float pwm =
        static_cast<float>(poll.rc[rc_config_.throttle_channel]);
    if (override_active && throttle_masked) {
      if (last_trusted_throttle_valid_) {
        out.throttle_pwm = static_cast<float>(last_trusted_throttle_pwm_);
        out.throttle_norm = last_trusted_throttle_norm_;
      } else {
        out.throttle_pwm = 0.0F;
        out.throttle_norm = std::numeric_limits<float>::quiet_NaN();
      }
    } else {
      out.throttle_pwm = pwm;
      out.throttle_norm = normalizeThrottlePwm(pwm, rc_config_);
      last_trusted_throttle_pwm_ =
          clampRcPwm(static_cast<uint16_t>(std::lround(pwm)));
      last_trusted_throttle_norm_ = out.throttle_norm;
      last_trusted_throttle_valid_ = true;
    }
    out.valid = true;
    last_rc_ok_at_ = now;
  } else if (!poll.rc_requested && override_active) {
    // RC intentionally skipped during OVERRIDE: reuse the trusted throttle
    // captured before/while engaging so FcState stays well-formed.
    if (last_trusted_throttle_valid_) {
      out.throttle_pwm = static_cast<float>(last_trusted_throttle_pwm_);
      out.throttle_norm = last_trusted_throttle_norm_;
    } else {
      out.throttle_norm = std::numeric_limits<float>::quiet_NaN();
    }
  }

  // Health flags reflect "ever received a good frame", not this cycle's request
  // set. Only flip to true; never back to false just because a cycle skipped it.
  if (status_ok) {
    last_poll_status_ok_ = true;
  }
  if (attitude_ok) {
    last_poll_attitude_ok_ = true;
  }
  if (poll.rc_ok) {
    last_poll_rc_ok_ = true;
  }
  have_fc_state_ = true;

  // ok/fail health: with the async pipeline a frame requested THIS cycle
  // normally replies a cycle later, so "requested but not yet parsed" must NOT
  // count as a failure. A requested frame is healthy if it arrived now OR its
  // cached value is still fresh (reply merely in-flight). Only a genuinely
  // stale-and-missing requested frame is a real failure.
  constexpr double kHealthyAgeMs = 300.0;
  const auto ageMs = [&](std::chrono::steady_clock::time_point t) -> double {
    if (t.time_since_epoch().count() == 0) {
      return std::numeric_limits<double>::max();
    }
    return std::chrono::duration<double, std::milli>(now - t).count();
  };
  const bool status_healthy = !poll.status_requested || status_ok ||
                              ageMs(last_status_ok_at_) < kHealthyAgeMs;
  const bool attitude_healthy = !poll.attitude_requested || attitude_ok ||
                                ageMs(last_attitude_ok_at_) < kHealthyAgeMs;
  const bool rc_healthy = !poll.rc_requested || poll.rc_ok ||
                          ageMs(last_rc_ok_at_) < kHealthyAgeMs;
  if (status_healthy && attitude_healthy && rc_healthy) {
    ++poll_ok_;
    last_poll_error_.clear();
  } else {
    ++poll_fail_;
    last_poll_error_ = client_->lastError();
  }
  return out;
}

circle::types::FcState BfStateSource::snapshot() const {
  circle::types::FcState out;
  out.stamp_ns = circle::types::monotonicNowNs();
  std::lock_guard<std::mutex> lk(mu_);
  MspAttitude att{};
  MspStatus status{};
  bool override_active = false;
  const bool status_ok = client_->readStatus(status);
  if (status_ok) {
    out.armed = status.armed;
    out.valid = true;
    override_active =
        (status.mode_flags & override_mode_flag_) != 0U;
    last_mode_flags_ = status.mode_flags;
  }
  override_active_ = override_active;

  const bool attitude_ok = client_->readAttitude(att);
  if (attitude_ok) {
    out.roll_rad = static_cast<float>(att.roll_deci_deg) * 0.1F *
                   static_cast<float>(M_PI) / 180.0F;
    out.pitch_rad = static_cast<float>(att.pitch_deci_deg) * 0.1F *
                    static_cast<float>(M_PI) / 180.0F;
    out.yaw_rad = static_cast<float>(att.yaw_deci_deg) * 0.1F *
                  static_cast<float>(M_PI) / 180.0F;
    out.valid = true;
  }

  const bool throttle_masked =
      (override_channels_mask_ &
       (1U << rc_config_.throttle_channel)) != 0U;
  std::vector<uint16_t> rc_channels;
  const bool rc_read_ok = client_->readRc(&rc_channels);
  const bool rc_ok =
      rc_read_ok && rc_config_.throttle_channel < rc_channels.size();
  if (rc_ok) {
    const float pwm =
        static_cast<float>(rc_channels[rc_config_.throttle_channel]);
    if (override_active && throttle_masked) {
      // BF MSP_RC for OVERRIDE-masked channels is latched/garbage; do not use.
      if (last_trusted_throttle_valid_) {
        out.throttle_pwm = static_cast<float>(last_trusted_throttle_pwm_);
        out.throttle_norm = last_trusted_throttle_norm_;
      } else {
        out.throttle_pwm = 0.0F;
        out.throttle_norm = std::numeric_limits<float>::quiet_NaN();
      }
    } else {
      out.throttle_pwm = pwm;
      out.throttle_norm = normalizeThrottlePwm(pwm, rc_config_);
      last_trusted_throttle_pwm_ =
          clampRcPwm(static_cast<uint16_t>(std::lround(pwm)));
      last_trusted_throttle_norm_ = out.throttle_norm;
      last_trusted_throttle_valid_ = true;
    }
    out.valid = true;
  }

  last_poll_status_ok_ = status_ok;
  last_poll_attitude_ok_ = attitude_ok;
  last_poll_rc_ok_ = rc_ok;
  if (status_ok && attitude_ok && rc_ok) {
    ++poll_ok_;
    last_poll_error_.clear();
  } else {
    ++poll_fail_;
    last_poll_error_ = client_->lastError();
  }
  return out;
}

}  // namespace circle::bf
