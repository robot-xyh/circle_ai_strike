#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "circle/bf/bf_flight_config_yaml.hpp"
#include "circle/bf/bf_rc_mapper.hpp"
#include "circle/bf/msp_client.hpp"
#include "circle/bf/runtime/bf_strike_controller_iface.hpp"
#include "circle/perception/camera_source.hpp"
#include "circle/types/detection.hpp"
#include "circle/vision/detection_filter.hpp"

namespace circle::bf::runtime {

/**
 * Controller-agnostic runtime configuration: everything bf_flight needed
 * outside of the controller's own params. Reuses circle_bf_common's
 * BfFlightMspConfig + BfRcMapperConfig and adds the perception / watchdog /
 * handover / coast / engage-gate / preview fields plus camera + intrinsics.
 */
struct BfRuntimeConfig {
  std::string camera_yaml;
  std::string camera_info_yaml;
  std::string model_path;
  std::string detection_file;
  std::string log_level{"info"};
  std::string log_color{"auto"};
  std::string mode_tag{"target_strike"};
  uint32_t max_infer_fps{0};
  bool zero_copy_pipeline_enabled{true};
  int zero_copy_slot_count{3};
  int infer_worker_count{1};
  std::vector<int> rknn_core_masks{};
  float conf_threshold{0.25F};
  float iou_threshold{0.45F};
  int max_det{300};
  double detection_coast_s{0.25};
  bool engage_require_fresh_detection{true};
  double engage_detection_fresh_timeout_s{0.30};
  bool watchdog_enabled{true};
  double watchdog_state_timeout_s{0.5};
  double watchdog_overrun_warn_s{0.25};
  double throttle_handover_s{0.4};
  // Control-loop cadence (Hz). Independent of msp.live_publish_hz (MSP wire
  // rate): the control loop computes/stages commands and publishes telemetry at
  // this rate; the MSP IO thread sends the latest staged command at live rate.
  // Clamped to [kControlLoopHzMin, kControlLoopHzMax] in the host.
  double control_loop_hz{200.0};
  bool preview_shm_enabled{true};
  uint32_t preview_max_fps{12};

  // Controller-related flags the host needs for gating / passthrough.
  bool dry_run{false};
  bool require_armed_to_command{true};

  circle::bf::BfFlightMspConfig msp{};
  circle::bf::BfRcMapperConfig rc{};
  // Detection filter for the vision pipeline (bf_flight: strike.filter; png:
  // built from node-level min_score / target_class_name).
  circle::vision::DetectionFilterParams filter{};
  circle::perception::MppCameraSourceConfig camera{};
  circle::types::CameraIntrinsics intrinsics{};
};

#if defined(CIRCLE_STRIKE_HAS_YAML) && CIRCLE_STRIKE_HAS_YAML
/**
 * Parse the shared `bf_flight:` block (+ resolve relative camera/model/info
 * paths against the package root) into a BfRuntimeConfig. The controller's own
 * params (`strike:` / `strike_png:`) are loaded separately by each executable.
 */
void loadBfRuntimeConfigFromYaml(const std::string& path, BfRuntimeConfig& cfg);
#endif

/**
 * Shared BF control host. Owns MSP open/probe/autobaud/BOXIDS detection, the
 * BfRateSink/BfStateSource, the perception pipeline (camera/infer/preview), the
 * dedicated MSP I/O thread (staged-wire SET_RAW_RC + interleaved polls), the
 * watchdog, throttle handover, physical hold, passthrough and all telemetry /
 * preview / param SHM. The concrete controller plugs in via IBfStrikeController.
 */
class BfControlHost {
 public:
  BfControlHost(BfRuntimeConfig config, std::shared_ptr<circle::bf::MspClient> msp,
                IBfStrikeController& controller);
  ~BfControlHost();

  BfControlHost(const BfControlHost&) = delete;
  BfControlHost& operator=(const BfControlHost&) = delete;

  /** Run the full stack until `running` is cleared. Returns process exit code.
   *  `max_iterations` > 0 limits the control loop (self-test / smoke). */
  int run(std::atomic<bool>& running, uint64_t max_iterations = 0);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace circle::bf::runtime
