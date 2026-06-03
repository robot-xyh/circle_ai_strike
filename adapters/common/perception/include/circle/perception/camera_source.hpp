#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <string>

#include "circle/types/detection.hpp"
#include "circle/perception/mpp_rga_pipeline.hpp"

namespace circle::perception {

struct FrameReady {
  uint64_t seq{0};
  circle::types::TimestampNs capture_ns{0};
  const uint8_t* data{nullptr};
  size_t data_size{0};
  int fd{-1};
  uint32_t width{0};
  uint32_t height{0};
  /** True when data is decoded RGB (pipeline output), not raw MJPEG. */
  bool hw_rgb{false};
  float capture_time_ms{0.0F};
  float decode_time_ms{0.0F};
};

class ICameraSource {
 public:
  virtual ~ICameraSource() = default;

  virtual bool start() = 0;
  virtual void stop() = 0;
  virtual bool grab(FrameReady& out) = 0;
};

/**
 * UVC V4L2 controls applied at openV4L2() time. Mirrors mpp_cam_node's
 * uvc_controls.cpp behavior without ROS dependency. For each numeric control
 * the raw value (-1 = skip) wins when `uvc_control_mode == "raw"`; in
 * "normalized" mode the *_pct value (1..100, mapped onto driver-reported
 * min/max via VIDIOC_QUERYCTRL) wins instead. Boolean toggles
 * (autoexposure/auto_white_balance/autofocus) gate whether the related raw
 * controls are touched at all.
 */
struct UvcControlsConfig {
  /** "raw" or "normalized". */
  std::string mode{"raw"};

  int brightness{-1};
  int contrast{-1};
  int saturation{-1};
  int sharpness{-1};
  int gain{-1};
  int gamma{-1};
  int backlight_compensation{-1};
  int exposure{-1};
  int white_balance{-1};

  int brightness_pct{-1};
  int contrast_pct{-1};
  int saturation_pct{-1};
  int sharpness_pct{-1};
  int gain_pct{-1};
  int gamma_pct{-1};
  int backlight_compensation_pct{-1};
  int exposure_pct{-1};
  int white_balance_pct{-1};

  bool autoexposure{true};
  bool auto_white_balance{true};
  bool autofocus{false};
};

struct MppCameraSourceConfig {
  std::string device{"/dev/video0"};
  uint32_t width{0};
  uint32_t height{0};
  uint32_t fps{0};
  uint32_t output_width{0};
  uint32_t output_height{0};
  UvcControlsConfig uvc{};
};

// V4L2-backed capture source. It keeps Profile B/C ABI stable across
// `mpp_cam_node` and `bf_flight` by exposing a single `grab()` entry point.
// Runtime may still run in stub mode if camera open fails.
class MppCameraSource final : public ICameraSource {
 public:
  explicit MppCameraSource(MppCameraSourceConfig config = {});

  bool start() override;
  void stop() override;
  bool grab(FrameReady& out) override;

 private:
  struct CapturedFrame {
    void* data{nullptr};
    size_t size{0};
    uint32_t index{0};
  };

  bool openV4L2();
  void closeV4L2();
  bool captureMjpegFrame(CapturedFrame& out_frame);
  bool decodeMjpegToRgb(const uint8_t* mjpeg, size_t mjpeg_size, FrameReady& out);

  MppCameraSourceConfig config_;
  MppRgaPipeline pipeline_;
  std::vector<uint8_t> rgb_scratch_;
  bool running_{false};
  int v4l2_fd_{-1};
  std::vector<void*> v4l2_buffers_;
  std::vector<size_t> v4l2_buffer_sizes_;
  int v4l2_buffer_count_{0};
  bool v4l2_streaming_{false};
  int32_t active_buffer_index_{-1};
  uint64_t seq_{0};
};

}  // namespace circle::perception
