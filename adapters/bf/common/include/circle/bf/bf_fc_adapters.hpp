#pragma once

#include <chrono>
#include <memory>
#include <mutex>

#include "circle/bf/bf_rc_mapper.hpp"
#include "circle/bf/msp_client.hpp"
#include "circle/fc_iface/rate_sink.hpp"
#include "circle/fc_iface/vehicle_state_source.hpp"
#include "circle/types/fc_state.hpp"
#include "circle/types/rate_command.hpp"

namespace circle::bf {

struct MspPassthroughDebugConfig {
  bool log_enabled{true};
  double log_interval_s{1.0};
  uint16_t throttle_jump_pwm{40};
  /** BF mode_flags bit when MSP OVERRIDE is active (board-specific; 0x2000000 typical). */
  uint32_t override_mode_flag{0x2000000U};
  /** Matches BF msp_override_channels_mask (bit i -> RC index i). Default 15 = R/P/Y/T. */
  uint32_t override_channels_mask{0x0FU};
  /** Passthrough SET_RAW_RC width; BF may expose 16ch via MSP_RC. */
  uint16_t passthrough_channel_count{16};
  /** dry_run=false live mapRates publish rate (BF UART limit). */
  double live_publish_hz{50.0};
  /** After raw OVERRIDE OFF, keep sending latched MSP SET_RAW_RC for this long (s). */
  double override_grace_hold_s{0.35};
};

class BfRateSink final : public circle::fc_iface::IRateSink {
 public:
  BfRateSink(std::shared_ptr<MspClient> client, BfRcMapper mapper,
             MspPassthroughDebugConfig passthrough_debug = {});

  void publishRates(const circle::types::RateCommand& command,
                    const circle::types::SafetyContext& safety) override;

  /**
   * Scheme B: compute the SET_RAW_RC wire from the LATEST cached MSP state (no
   * serial I/O, no rate gate) and stage it for the single MSP I/O thread to
   * flush. Called from the control loop every iteration.
   */
  void stageWire(const circle::types::RateCommand& command,
                 const circle::types::SafetyContext& safety);
  /** Stage the disarmed/over­ride physical-stick hold wire (no serial I/O). */
  void stagePhysicalHold();
  /**
   * Flush the most recently staged wire as a fire-and-forget SET_RAW_RC. Called
   * by the MSP I/O thread FIRST in each cycle so SET has top priority. Returns
   * true if a frame was written this call.
   */
  bool flushStagedWire();

  /** dry_run bench: MSP_RC → MSP_SET_RAW_RC passthrough for OVERRIDE handoff. */
  bool publishRcPassthrough();

  /**
   * When dry_run=false and BF MSP OVERRIDE is active: throttle follows MSP_RC
   * unless command.thrust_z > 0 (algorithm hover / tracking / etc.).
   */

  /** Last MSP transact error after a failed publishRcPassthrough(). */
  const std::string& lastPassthroughError() const { return last_passthrough_error_; }

  struct LastSentThrottle {
    uint16_t pwm{0};
    float norm{0.0F};
    bool valid{false};
  };

  struct LastSentWire {
    uint16_t roll_pwm{0};
    uint16_t pitch_pwm{0};
    uint16_t yaw_pwm{0};
    uint16_t throttle_pwm{0};
    float roll_norm{0.0F};
    float pitch_norm{0.0F};
    bool valid{false};
  };

  /** Throttle on MSP_SET_RAW_RC wire (AETR index 2) after last successful send. */
  LastSentThrottle lastSentThrottle() const;
  LastSentWire lastSentWire() const;
  uint64_t wirePublishCount() const;
  bool mspOverrideActive() const;
  /** Raw OVERRIDE bit from the last MSP_STATUS read (no debounce/grace). */
  bool mspOverrideRawActive() const;
  /** True while grace-hold continues sending latched MSP after debounced OFF. */
  bool mspOverrideGraceHoldActive() const;
  /** Raw BF MSP_STATUS mode_flags (0 when status invalid). For bit diagnosis. */
  uint32_t cachedModeFlags() const;
  /** Configured override_mode_flag bitmask we test mode_flags against. */
  uint32_t overrideModeFlag() const;
  /** Poll MSP status/RC cache (e.g. with vehicle snapshot); safe when disarmed. */
  void refreshMspStatusFromPoll();
  /**
   * Feed a batched poll result (status + optional RC) into the override
   * debounce/latch state machine WITHOUT issuing any serial reads. Used by the
   * dedicated MSP poll thread so status/RC are read once and shared.
   */
  void ingestPoll(const MspPollResult& poll);
  /**
   * Whether the next poll still needs RC. False once OVERRIDE is active AND the
   * physical-stick latch is captured (RC for masked channels is then garbage).
   */
  bool needsRcPoll() const;
  /** Physical throttle captured at OVERRIDE rising edge (same as dry_run latch). */
  LastSentThrottle lastLatchedPhysicalThrottle() const;
  /**
   * dry_run=false + OVERRIDE + disarmed: keep sending latched physical sticks
   * (same wire path as dry_run passthrough).
   */
  void publishOverridePhysicalHold();

  void resetActivationLatch() override {}

 private:
  void logPassthrough(const MspStatus& status,
                      const std::vector<uint16_t>& read_channels,
                      const std::vector<uint16_t>& sent_channels,
                      bool override_latch_active,
                      bool force) const;

  std::shared_ptr<MspClient> client_;
  BfRcMapper mapper_;
  MspPassthroughDebugConfig passthrough_debug_;
  mutable std::mutex mu_;
  std::string last_passthrough_error_;
  mutable uint32_t last_mode_flags_{0};
  mutable uint16_t last_throttle_read_{0};
  mutable std::chrono::steady_clock::time_point last_log_at_{};
  std::vector<uint16_t> last_physical_channels_;
  std::vector<uint16_t> last_rc_read_normalized_;
  std::vector<uint16_t> override_latched_channels_;
  bool override_was_active_{false};
  bool override_latch_valid_{false};
  bool cached_override_active_{false};
  bool cached_override_raw_active_{false};
  bool override_debounced_active_{false};
  bool override_grace_hold_active_{false};
  std::chrono::steady_clock::time_point override_grace_until_{};
  bool cached_status_valid_{false};
  MspStatus cached_status_{};
  std::chrono::steady_clock::time_point last_msp_context_at_{};
  uint16_t last_sent_throttle_pwm_{0};
  float last_sent_throttle_norm_{0.0F};
  bool last_sent_valid_{false};
  uint16_t last_sent_roll_pwm_{0};
  uint16_t last_sent_pitch_pwm_{0};
  uint16_t last_sent_yaw_pwm_{0};
  float last_sent_roll_norm_{0.0F};
  float last_sent_pitch_norm_{0.0F};
  uint64_t live_publish_ok_{0};
  uint64_t live_publish_fail_{0};
  bool live_override_warned_{false};

  // A fully-computed SET_RAW_RC frame plus the metadata needed to log it at
  // flush time. Produced by the control thread (computeWire), consumed by the
  // single MSP I/O thread (flushStagedWire).
  struct StagedSend {
    std::vector<uint16_t> wire;
    std::vector<uint16_t> sent_channels;
    circle::types::RateCommand command{};
    bool has_command{false};
    bool override_active{false};
    bool armed_ok{false};
    bool override_latch_active{false};
    bool algo_commanding{false};
    bool algo_rates_active{false};
    bool algo_throttle_active{false};
    bool passthrough_hold{false};
    bool valid{false};
  };
  StagedSend staged_{};
  uint64_t staged_seq_{0};
  uint64_t flushed_seq_{0};
  // Counts commands STAGED by the control loop (not serial sends). Exposed via
  // wirePublishCount() so the control loop's per-iteration latency plumbing
  // still sees "a command was emitted this iteration" under async sending.
  uint64_t staged_publish_count_{0};

  /** Build the live SET_RAW_RC wire from cached state; no serial, no gate. */
  bool computeWire(const circle::types::RateCommand& command,
                   const circle::types::SafetyContext& safety,
                   StagedSend& out);
  /** Build the override/disarmed physical-hold wire; no serial, no gate. */
  bool computePhysicalHold(StagedSend& out);

  void recordSentWire(const std::vector<uint16_t>& wire);
  void refreshMspContextLocked(bool force_refresh = false);
  /** Apply a status/RC sample (already read) to the override/latch machine. */
  void applyPollContextLocked(const MspStatus& status, bool status_ok,
                              const std::vector<uint16_t>& rc_channels,
                              bool rc_ok);
  bool applyOverrideDebouncing(bool raw_active);
  void updatePhysicalAndOverrideLatch(const std::vector<uint16_t>& normalized,
                                      bool effective_override_active);
  void logLivePublish(const std::vector<uint16_t>& wire, bool override_active,
                      bool send_ok, bool passthrough_hold,
                      const circle::types::RateCommand* command,
                      const std::vector<uint16_t>* logical_channels,
                      bool armed_ok, bool override_latch_active,
                      bool algo_commanding, bool algo_rates_active,
                      bool algo_throttle_active);
  void logLiveSkip(const char* reason, const circle::types::RateCommand& command,
                   bool override_active, bool armed_ok,
                   bool override_latch_active, bool algo_rates_active);
};

struct MspPollStats {
  uint64_t ok{0};
  uint64_t fail{0};
  /**
   * "have ever received a good frame of this type" — cached health, NOT the
   * per-cycle request presence. With alternating polls a given cycle may not
   * request STATUS/ATT/RC; these stay true so logs don't show misleading 0s.
   */
  bool last_status_ok{false};
  bool last_attitude_ok{false};
  bool last_rc_ok{false};
  /** ms since the last successfully parsed frame of each type (-1 = never). */
  double status_age_ms{-1.0};
  double attitude_age_ms{-1.0};
  double rc_age_ms{-1.0};
  std::string last_error;
  /** Cached BF MSP_STATUS mode_flags from the last good STATUS (not this cycle). */
  uint32_t mode_flags{0};
  /** Static configured mask we AND mode_flags against to detect MSP OVERRIDE. */
  uint32_t override_mode_flag{0};
  /** (mode_flags & override_mode_flag) != 0 from the last good STATUS. */
  bool override_active{false};
};

class BfStateSource final : public circle::fc_iface::IVehicleStateSource {
 public:
  BfStateSource(std::shared_ptr<MspClient> client, BfRcMapperConfig rc_config,
                uint32_t override_mode_flag = 0x2000000U,
                uint32_t override_channels_mask = 0x0FU);

  circle::types::FcState snapshot() const override;
  /**
   * Build an FcState from an already-fetched batched poll result (no serial
   * I/O), and update the internal poll stats / trusted-throttle caches. Used by
   * the dedicated MSP poll thread.
   */
  circle::types::FcState ingestPoll(const MspPollResult& poll) const;
  bool mspOverrideActive() const;
  MspPollStats pollStats() const;

 private:
  static float normalizeThrottlePwm(float pwm, const BfRcMapperConfig& cfg);

  std::shared_ptr<MspClient> client_;
  BfRcMapperConfig rc_config_;
  uint32_t override_mode_flag_{0x2000000U};
  uint32_t override_channels_mask_{0x0FU};
  mutable std::mutex mu_;
  mutable bool override_active_{false};
  mutable uint32_t last_mode_flags_{0};
  mutable uint16_t last_trusted_throttle_pwm_{0};
  mutable float last_trusted_throttle_norm_{0.0F};
  mutable bool last_trusted_throttle_valid_{false};
  mutable uint64_t poll_ok_{0};
  mutable uint64_t poll_fail_{0};
  mutable bool last_poll_status_ok_{false};
  mutable bool last_poll_attitude_ok_{false};
  mutable bool last_poll_rc_ok_{false};
  mutable std::string last_poll_error_;
  // Persistent merged state so a cycle that only carried STATUS (or only ATT)
  // never zeroes the other fields. Only frame types actually parsed this cycle
  // update their slice; everything else keeps the last good value.
  mutable circle::types::FcState last_fc_state_{};
  mutable bool have_fc_state_{false};
  mutable std::chrono::steady_clock::time_point last_status_ok_at_{};
  mutable std::chrono::steady_clock::time_point last_attitude_ok_at_{};
  mutable std::chrono::steady_clock::time_point last_rc_ok_at_{};
};

}  // namespace circle::bf
