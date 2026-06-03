#pragma once

#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace circle::bf {

enum class MspCommand : uint16_t {
  kStatus = 101,
  kRc = 105,
  kAttitude = 108,
  kBoxIds = 119,
  kSetRawRc = 200,
};

/** BF permanent box ID for the "MSP OVERRIDE" mode (msp_box.c, never reused). */
inline constexpr uint8_t kBoxPermanentIdMspOverride = 50;

struct MspAttitude {
  int16_t roll_deci_deg{0};
  int16_t pitch_deci_deg{0};
  int16_t yaw_deci_deg{0};
};

struct MspStatus {
  uint16_t cycle_time_us{0};
  uint16_t i2c_errors{0};
  uint16_t sensor_flags{0};
  uint32_t mode_flags{0};
  bool armed{false};
};

/**
 * One batched flight-state poll: STATUS + ATTITUDE (+ RC) requested and read in
 * a SINGLE serial round-trip (pipelined), instead of three sequential
 * request/response transactions. `rc_*` is only populated when RC was requested.
 */
struct MspPollResult {
  // *_requested: which query frames the caller asked for THIS cycle (for the
  // async/interleaved schedule). *_ok: which replies were actually parsed.
  bool status_requested{false};
  bool attitude_requested{false};
  bool rc_requested{false};
  bool status_ok{false};
  bool attitude_ok{false};
  bool rc_ok{false};
  MspStatus status{};
  MspAttitude attitude{};
  std::vector<uint16_t> rc;
};

/** Aggregated serial I/O timing over a logging window (poll + SET_RAW_RC). */
struct MspIoWindow {
  double window_s{0.0};
  uint64_t poll_ops{0};
  uint64_t send_ops{0};
  uint64_t poll_fail{0};
  uint64_t send_fail{0};
  uint64_t tx_bytes{0};
  uint64_t rx_bytes{0};
  /** Sum of lock-wait time before acquiring io_mu_ (poll/send contention). */
  uint64_t lock_wait_us{0};
  uint64_t max_lock_wait_us{0};
  /** Poll-only round-trip sums (write → last response byte). */
  uint64_t poll_round_trip_us{0};
  uint64_t poll_wire_us{0};
  uint64_t max_poll_round_trip_us{0};
  /** SET_RAW_RC transact round-trip sums. */
  uint64_t send_round_trip_us{0};
  uint64_t send_wire_us{0};
  uint64_t max_send_round_trip_us{0};
  std::chrono::steady_clock::time_point window_start{};
};

class MspClient {
 public:
  MspClient() = default;
  ~MspClient();

  MspClient(const MspClient&) = delete;
  MspClient& operator=(const MspClient&) = delete;

  bool openSerial(const std::string& device, int baudrate);
  void close();

  bool readAttitude(MspAttitude& out);
  bool readStatus(MspStatus& out);
  /** Active RC channels as BF sees them (receiver + overrides merged). */
  bool readRc(std::vector<uint16_t>* channels);
  /**
   * Pipelined poll: writes STATUS, ATTITUDE and (optionally) RC requests
   * back-to-back, then reads all replies within one deadline window. Collapses
   * 2-3 serial round-trips into ~1, which is the dominant control-loop latency.
   * Returns true if at least STATUS and ATTITUDE were parsed.
   */
  bool pollState(bool want_rc, MspPollResult& out);
  bool sendRawRc(const std::vector<uint16_t>& channels);

  /**
   * Async (fire-and-forget) request write: writes ONE MSP request frame and
   * returns immediately without waiting for a reply. Used by the single MSP I/O
   * thread so SET_RAW_RC has top priority and never blocks on poll round-trips.
   * The reply (if any) is collected later by drainResponses().
   */
  bool writeFrameRaw(MspCommand cmd, const uint8_t* payload, uint16_t len);
  /** Convenience: build + fire-and-forget MSP_SET_RAW_RC (no ack wait). */
  bool writeRawRc(const std::vector<uint16_t>& channels);
  /**
   * Non-blocking drain: read whatever bytes are currently available (bounded by
   * budget), parse all complete frames accumulated across cycles, and route
   * STATUS/ATTITUDE/RC into `out` (only the frame types that actually arrived
   * are marked *_ok). SET_RAW_RC acks are consumed for stats. Partial frames are
   * kept in an internal buffer for the next drain.
   */
  void drainResponses(std::chrono::steady_clock::duration budget,
                      MspPollResult& out);
  /**
   * MSP_BOXIDS (119): permanent box IDs in the SAME order BF packs MSP_STATUS
   * mode_flags bits. Index i in this list ⇒ bit i in mode_flags. Lets us derive
   * the MSP OVERRIDE bit deterministically instead of guessing a fixed mask.
   */
  bool readBoxIds(std::vector<uint8_t>* permanent_ids);

  /** Human-readable reason when the last transact/read failed. */
  const std::string& lastError() const { return last_error_; }

  int baudrate() const { return baud_; }

  /**
   * Returns accumulated I/O stats since the last call and starts a new window.
   * Safe to call from any thread; internally synchronized on io_mu_.
   */
  MspIoWindow snapshotIoWindow();

 private:
  enum class IoKind : uint8_t { kPoll, kSend };

  void recordIoSample(IoKind kind, uint32_t tx_bytes, uint32_t rx_bytes,
                      uint32_t round_trip_us, uint32_t wire_us,
                      uint32_t lock_wait_us, bool ok);

  static uint32_t wireTimeUs(uint32_t bytes, int baud);
  bool transact(MspCommand cmd,
                const uint8_t* payload,
                uint16_t payload_len,
                std::vector<uint8_t>& response);

  void setError(std::string msg);

  int fd_{-1};
  int baud_{115200};
  mutable std::mutex io_mu_;
  std::string last_error_;
  MspIoWindow io_window_{};

  // Async I/O path (single MSP I/O thread): persistent RX buffer for partial
  // frames across drains, and per-command request timestamps for round-trip.
  std::vector<uint8_t> drain_buf_;
  std::chrono::steady_clock::time_point req_at_set_{};
  std::chrono::steady_clock::time_point req_at_status_{};
  std::chrono::steady_clock::time_point req_at_attitude_{};
  std::chrono::steady_clock::time_point req_at_rc_{};
};

}  // namespace circle::bf
