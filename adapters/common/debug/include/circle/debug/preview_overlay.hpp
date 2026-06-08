#pragma once

#include <cstdint>
#include <cstring>
#include <limits>
#include <optional>
#include <string>
#include <type_traits>

#include <opencv2/core.hpp>

namespace circle::debug {

struct PreviewOverlayDetection {
  bool valid{false};
  float cx{0.0F};
  float cy{0.0F};
  float width{0.0F};
  float height{0.0F};
  float score{0.0F};
  int class_id{-1};
  std::string class_name;
  uint64_t seq{0};
  int64_t capture_ns{0};
  int track_id{-1};
  bool tracker_predicted{false};
  int tracker_lost_frames{0};
};

struct PreviewOverlayContext {
  std::string source{"debug"};
  std::string mode_tag{"target_strike"};
  std::string state;
  bool armed{false};
  bool has_target{false};

  uint64_t frame_seq{0};
  int64_t stamp_ns{0};
  std::optional<double> fps;

  PreviewOverlayDetection detection{};
  std::optional<float> pixel_offset_x;
  std::optional<float> pixel_offset_y;
  std::optional<float> image_ex;
  std::optional<float> image_ey;
  std::optional<float> roll_rate_rad_s;
  std::optional<float> pitch_rate_rad_s;
  std::optional<float> yaw_rate_rad_s;
  std::optional<float> thrust_z;
  std::optional<float> throttle_algo_norm;
  std::optional<float> throttle_cmd_norm;

  std::optional<float> vehicle_roll_rad;
  std::optional<float> vehicle_pitch_rad;
  std::optional<float> vehicle_yaw_rad;
  std::optional<float> vehicle_roll_rate_rad_s;
  std::optional<float> vehicle_pitch_rate_rad_s;
  std::optional<float> vehicle_yaw_rate_rad_s;
  std::optional<float> vehicle_throttle_pwm;
  std::optional<float> vehicle_throttle_norm;

  std::optional<float> cam_pipeline_ms;
  std::optional<float> capture_time_ms;
  std::optional<float> decode_time_ms;
  std::optional<float> preprocess_time_ms;
  std::optional<float> inference_time_ms;
  std::optional<float> postprocess_time_ms;
  std::optional<float> executor_ms;
  std::optional<float> e2e_ms;

  /** Pipeline perf HUD (bf_flight E2E metering). */
  bool perf_wire_path_active{false};
  bool perf_pipe_zero_copy{false};
  std::optional<float> perf_e2e_fps;
  std::optional<float> perf_prod_fps;
  std::optional<float> perf_inf_fps;
  std::optional<float> perf_ctrl_fps;
  std::optional<float> perf_msp_fps;
  std::optional<float> perf_wait_grab_ms;
  std::optional<float> perf_e2e_input_ms;
  std::optional<float> perf_e2e_input_p50_ms;
  std::optional<float> perf_e2e_wire_ms;
  std::optional<float> perf_e2e_wire_p50_ms;
  std::optional<float> perf_e2e_algo_ms;
  std::optional<float> perf_e2e_algo_p50_ms;
  std::optional<float> perf_queue_wait_ms;
  std::optional<float> perf_producer_ms;
  std::optional<float> perf_cnn_ms;
  std::optional<float> perf_ctrl_ms;
  std::optional<float> perf_msp_gate_ms;
  std::optional<int> perf_pipe_slot_count;
  std::optional<uint64_t> perf_pipe_slot_busy;
  std::optional<uint64_t> perf_pipe_ready_drop;

  std::optional<float> deadband_half_w_px;
  std::optional<float> deadband_half_h_px;
  std::optional<float> aim_offset_x_px;
  std::optional<float> aim_offset_y_px;
};

/** SHM-safe POD struct mirroring PreviewOverlayDetection. */
struct PreviewOverlayDetectionShmData {
  uint8_t valid{0};
  float cx{0.0F};
  float cy{0.0F};
  float width{0.0F};
  float height{0.0F};
  float score{0.0F};
  int32_t class_id{-1};
  char class_name[64]{};
  uint64_t seq{0};
  int64_t capture_ns{0};
  int32_t track_id{-1};
  uint8_t tracker_predicted{0};
  int32_t tracker_lost_frames{0};
};

/** SHM-safe POD struct mirroring PreviewOverlayContext.
 *  All std::optional<float> → float with NaN sentinel.
 *  All std::optional<double> → double with NaN sentinel.
 *  All std::optional<int> → int32_t with INT32_MIN sentinel.
 *  All std::optional<uint64_t> → uint64_t with 0 sentinel.
 *  All std::string → fixed-size char[].
 *  All bool → uint8_t. */
struct PreviewOverlayShmData {
  char source[16]{};
  char mode_tag[32]{};
  char state[32]{};
  uint8_t armed{0};
  uint8_t has_target{0};
  uint8_t perf_wire_path_active{0};
  uint8_t perf_pipe_zero_copy{0};

  uint64_t frame_seq{0};
  int64_t stamp_ns{0};
  double fps{std::numeric_limits<double>::quiet_NaN()};

  PreviewOverlayDetectionShmData detection{};

  float pixel_offset_x{std::numeric_limits<float>::quiet_NaN()};
  float pixel_offset_y{std::numeric_limits<float>::quiet_NaN()};
  float image_ex{std::numeric_limits<float>::quiet_NaN()};
  float image_ey{std::numeric_limits<float>::quiet_NaN()};
  float roll_rate_rad_s{std::numeric_limits<float>::quiet_NaN()};
  float pitch_rate_rad_s{std::numeric_limits<float>::quiet_NaN()};
  float yaw_rate_rad_s{std::numeric_limits<float>::quiet_NaN()};
  float thrust_z{std::numeric_limits<float>::quiet_NaN()};
  float throttle_algo_norm{std::numeric_limits<float>::quiet_NaN()};
  float throttle_cmd_norm{std::numeric_limits<float>::quiet_NaN()};

  float vehicle_roll_rad{std::numeric_limits<float>::quiet_NaN()};
  float vehicle_pitch_rad{std::numeric_limits<float>::quiet_NaN()};
  float vehicle_yaw_rad{std::numeric_limits<float>::quiet_NaN()};
  float vehicle_roll_rate_rad_s{std::numeric_limits<float>::quiet_NaN()};
  float vehicle_pitch_rate_rad_s{std::numeric_limits<float>::quiet_NaN()};
  float vehicle_yaw_rate_rad_s{std::numeric_limits<float>::quiet_NaN()};
  float vehicle_throttle_pwm{std::numeric_limits<float>::quiet_NaN()};
  float vehicle_throttle_norm{std::numeric_limits<float>::quiet_NaN()};

  float cam_pipeline_ms{std::numeric_limits<float>::quiet_NaN()};
  float capture_time_ms{std::numeric_limits<float>::quiet_NaN()};
  float decode_time_ms{std::numeric_limits<float>::quiet_NaN()};
  float preprocess_time_ms{std::numeric_limits<float>::quiet_NaN()};
  float inference_time_ms{std::numeric_limits<float>::quiet_NaN()};
  float postprocess_time_ms{std::numeric_limits<float>::quiet_NaN()};
  float executor_ms{std::numeric_limits<float>::quiet_NaN()};
  float e2e_ms{std::numeric_limits<float>::quiet_NaN()};

  float perf_e2e_fps{std::numeric_limits<float>::quiet_NaN()};
  float perf_prod_fps{std::numeric_limits<float>::quiet_NaN()};
  float perf_inf_fps{std::numeric_limits<float>::quiet_NaN()};
  float perf_ctrl_fps{std::numeric_limits<float>::quiet_NaN()};
  float perf_msp_fps{std::numeric_limits<float>::quiet_NaN()};
  float perf_wait_grab_ms{std::numeric_limits<float>::quiet_NaN()};
  float perf_e2e_input_ms{std::numeric_limits<float>::quiet_NaN()};
  float perf_e2e_input_p50_ms{std::numeric_limits<float>::quiet_NaN()};
  float perf_e2e_wire_ms{std::numeric_limits<float>::quiet_NaN()};
  float perf_e2e_wire_p50_ms{std::numeric_limits<float>::quiet_NaN()};
  float perf_e2e_algo_ms{std::numeric_limits<float>::quiet_NaN()};
  float perf_e2e_algo_p50_ms{std::numeric_limits<float>::quiet_NaN()};
  float perf_queue_wait_ms{std::numeric_limits<float>::quiet_NaN()};
  float perf_producer_ms{std::numeric_limits<float>::quiet_NaN()};
  float perf_cnn_ms{std::numeric_limits<float>::quiet_NaN()};
  float perf_ctrl_ms{std::numeric_limits<float>::quiet_NaN()};
  float perf_msp_gate_ms{std::numeric_limits<float>::quiet_NaN()};
  int32_t perf_pipe_slot_count{INT32_MIN};
  uint64_t perf_pipe_slot_busy{0};
  uint64_t perf_pipe_ready_drop{0};

  float deadband_half_w_px{std::numeric_limits<float>::quiet_NaN()};
  float deadband_half_h_px{std::numeric_limits<float>::quiet_NaN()};
  float aim_offset_x_px{std::numeric_limits<float>::quiet_NaN()};
  float aim_offset_y_px{std::numeric_limits<float>::quiet_NaN()};
};

static_assert(std::is_trivially_copyable_v<PreviewOverlayDetectionShmData>);
static_assert(std::is_trivially_copyable_v<PreviewOverlayShmData>);
static_assert(sizeof(PreviewOverlayShmData) < 2048);

PreviewOverlayShmData fromPreviewOverlayContext(const PreviewOverlayContext& ctx);
PreviewOverlayContext toPreviewOverlayContext(const PreviewOverlayShmData& data);

void drawPreviewOverlay(cv::Mat& bgr, const PreviewOverlayContext& ctx);

}  // namespace circle::debug
