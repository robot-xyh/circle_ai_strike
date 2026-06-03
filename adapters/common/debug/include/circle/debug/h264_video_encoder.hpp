#pragma once

#include <cstdint>
#include <string>

#include <opencv2/core.hpp>

namespace circle::debug {

struct EncodedH264Frame {
  std::string codec{"H264"};
  std::string data;
  int width{0};
  int height{0};
  int64_t timestamp_ns{0};
  bool keyframe{false};
};

/**
 * Debug-video H.264 encoder facade.
 *
 * The actual hardware encoder is provided by the deployment image
 * (RKMPP/VAAPI/NVENC through GStreamer). This wrapper keeps feature modules
 * independent from the concrete platform backend and reports a hard error
 * when the required backend is not available.
 */
class H264VideoEncoder {
 public:
  struct Config {
    std::string backend{"h264_platform_auto"};
    int bitrate_kbps{1800};
    double fps{10.0};
  };

  explicit H264VideoEncoder(Config config);

  bool isAvailable() const { return available_; }
  const std::string& backendName() const { return backend_name_; }
  const std::string& status() const { return status_; }

  bool encodeFrame(const cv::Mat& bgr, int64_t timestamp_ns,
                   EncodedH264Frame& out);

  void setFps(double fps) {
    config_.fps = fps > 0.1 ? fps : 10.0;
  }

 private:
  static bool commandSucceeds(const std::string& command);
  static std::string detectPlatformBackend();

  Config config_;
  std::string backend_name_;
  std::string status_;
  bool available_{false};
};

}  // namespace circle::debug
