#include "circle/debug/h264_video_encoder.hpp"

#include <cstdlib>

namespace circle::debug {

H264VideoEncoder::H264VideoEncoder(Config config)
    : config_(config) {
  backend_name_ = config_.backend;
  if (backend_name_ == "h264_platform_auto") {
    backend_name_ = detectPlatformBackend();
  }

  if (backend_name_.empty()) {
    status_ = "no supported H.264 hardware encoder detected";
    available_ = false;
    return;
  }

  // The full WebRTC/GStreamer sender needs both development headers (for this
  // library) and runtime plugins. `webrtcbin` uses libnice for ICE; without
  // gstreamer1.0-nice it cannot create RTP sink pads.
  const bool have_dev = commandSucceeds(
      "pkg-config --exists gstreamer-1.0 gstreamer-app-1.0 "
      "gstreamer-sdp-1.0 gstreamer-webrtc-1.0");
  const bool have_webrtcbin = commandSucceeds("gst-inspect-1.0 webrtcbin");
  const bool have_nice = commandSucceeds("gst-inspect-1.0 nice");
  available_ = have_dev && have_webrtcbin && have_nice;
  if (available_) {
    status_ = "H.264 backend ready: " + backend_name_;
  } else if (!have_nice) {
    status_ =
        "GStreamer libnice plugin missing; install gstreamer1.0-nice for "
        "WebRTC ICE transport";
  } else if (!have_webrtcbin) {
    status_ =
        "GStreamer webrtcbin plugin missing; install gstreamer1.0-plugins-bad";
  } else {
    status_ =
        "H.264/WebRTC development files missing; install gstreamer-webrtc-1.0 "
        "dev package for backend " +
        backend_name_;
  }
}

bool H264VideoEncoder::encodeFrame(const cv::Mat& bgr, int64_t timestamp_ns,
                                   EncodedH264Frame& out) {
  if (!available_) {
    return false;
  }
  // The actual hardware encoder pipeline is intentionally isolated behind
  // this method. When GStreamer WebRTC support is added to the build image,
  // this will push the already-overlaid BGR/NV12 frame into appsrc.
  out.width = bgr.cols;
  out.height = bgr.rows;
  out.timestamp_ns = timestamp_ns;
  out.keyframe = false;
  out.data.clear();
  return false;
}

bool H264VideoEncoder::commandSucceeds(const std::string& command) {
  const int rc = std::system((command + " >/dev/null 2>&1").c_str());
  return rc == 0;
}

std::string H264VideoEncoder::detectPlatformBackend() {
  if (commandSucceeds("gst-inspect-1.0 mpph264enc")) {
    return "h264_gstreamer_mpp";
  }
  if (commandSucceeds("gst-inspect-1.0 vaapih264enc")) {
    return "h264_gstreamer_vaapi";
  }
  if (commandSucceeds("gst-inspect-1.0 nvh264enc")) {
    return "h264_gstreamer_nvenc";
  }
  return {};
}

}  // namespace circle::debug
