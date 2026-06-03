#pragma once

#include <cstdint>
#include <string>

#include "circle/debug/preview_overlay.hpp"
#include "circle/ipc/strike_telemetry_shm.hpp"
#include "circle/types/detection.hpp"
#include "circle/types/fc_state.hpp"
#include "circle/types/rate_command.hpp"

namespace circle::bf::runtime {

/**
 * Per-tick inputs the host hands to a controller. The host owns detection coast
 * filling, MSP state caching, engage detection and watchdog; the controller
 * only consumes the assembled context.
 */
struct BfControlContext {
  uint64_t now_ns{0};
  circle::types::FrameDetection detection{};  // host already applied coast
  circle::types::FcState vehicle{};            // cached MSP snapshot
  bool mode_active{false};                     // MSP OVERRIDE active || dry_run
  bool override_active{false};
  circle::types::CameraIntrinsics intrinsics{};
  uint32_t image_width{0};                      // pipeline output width
  uint32_t image_height{0};                     // pipeline output height
};

/** Controller -> host result for one tick. */
struct BfControlResult {
  enum class Command { None, Algorithm, LevelOnly };

  circle::types::RateCommand rates{};
  circle::types::SafetyContext safety{};
  Command command{Command::None};
  bool has_target{false};
  float image_ex{0.0F};
  float image_ey{0.0F};
  /** Opaque per-controller state code (e.g. StrikeState int) for telemetry. */
  int state_code{0};
};

/**
 * Controller seam for the shared BF runtime. The host keeps ownership of the
 * throttle handover, watchdog, gating, MSP wire, passthrough, physical hold and
 * all SHM writes; concrete controllers only produce rates/command and fill the
 * controller-specific telemetry/overlay fields.
 */
class IBfStrikeController {
 public:
  virtual ~IBfStrikeController() = default;

  /** Called on the not-engaged -> engaged rising edge (reset + snapshots). */
  virtual void onEngageRisingEdge(const BfControlContext& ctx) = 0;

  /** Run one control tick. Implementations cache their last output so the
   *  fill* hooks below can read it. */
  virtual BfControlResult update(const BfControlContext& ctx) = 0;

  /** Online tuning: apply one {"name":...,"value":...} JSON update. */
  virtual bool applyParamUpdateJson(const std::string& json) = 0;
  /** Current params as the /api/params.json snapshot body. */
  virtual std::string paramsSnapshotJson() const = 0;

  /** Fill controller-specific telemetry fields (host fills the common ones). */
  virtual void fillTelemetry(circle::ipc::StrikeTelemetrySample& sample,
                             const BfControlResult& result) const = 0;
  /** Fill controller-specific overlay fields (host fills the common ones). */
  virtual void fillOverlay(circle::debug::PreviewOverlayContext& overlay,
                           const BfControlResult& result) const = 0;

  /** Hover throttle as a normalized [0,1] BF stick fraction. Used by the host as
   *  a safe floor / fallback for the throttle handover blend start. */
  virtual float hoverThrottleNorm() const = 0;

  /** Overlay/telemetry mode tag, e.g. "target_strike" / "target_strike_png". */
  virtual const char* modeTag() const = 0;
  /** Human-readable state name for the current cached output. */
  virtual const char* stateName(const BfControlResult& result) const = 0;
};

}  // namespace circle::bf::runtime
