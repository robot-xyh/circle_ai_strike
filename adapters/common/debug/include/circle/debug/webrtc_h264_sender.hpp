#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include <opencv2/core.hpp>

namespace circle::debug {

/**
 * GStreamer webrtcbin + hardware H.264 sender: browser POSTs SDP offer, we answer.
 * BGR frames are pushed via appsrc. Requires GStreamer WebRTC plugins at runtime.
 */
class WebRtcH264Sender {
 public:
  struct Stats {
    uint64_t frames_accepted{0};
    uint64_t frames_dropped{0};
    uint64_t frames_pushed{0};
    uint64_t bytes_pushed{0};
    uint64_t push_errors{0};
    uint64_t h264_buffers{0};
    uint64_t rtp_packets{0};
    int frame_width{0};
    int frame_height{0};
    int last_flow_return{0};
    double last_queue_wait_ms{0.0};
    double last_convert_ms{0.0};
    double last_push_ms{0.0};
    double max_queue_wait_ms{0.0};
    double max_convert_ms{0.0};
    double max_push_ms{0.0};
  };

  struct Config {
    /** Factory name, e.g. vaapih264enc, nvh264enc, mpph264enc. */
    std::string encoder_element;
    int bitrate_kbps{1800};
    double fps{10.0};
    /** Fallback when no frame received yet (multiples of 16 help encoders). */
    int default_width{960};
    int default_height{540};
    /** STUN server. Empty (default) = LAN-only host candidates, fastest path
     *  when both peers share the same broadcast domain. Set to e.g.
     *  "stun://stun.l.google.com:19302" or a self-hosted coturn when NAT
     *  traversal is needed. */
    std::string stun_server;
  };

  explicit WebRtcH264Sender(Config config);
  WebRtcH264Sender(const WebRtcH264Sender&) = delete;
  WebRtcH264Sender& operator=(const WebRtcH264Sender&) = delete;
  ~WebRtcH264Sender();

  bool start();
  void stop();
  bool isRunning() const { return running_; }

  /**
   * Browser JSON: {"type":"offer","sdp":"..."} (SDP may use \n escapes).
   * Returns {"ok":true,"type":"answer","sdp":"..."} or {"ok":false,"error":"..."}.
   */
  std::string handleBrowserOffer(const std::string& offer_json,
                                 const std::string& client_ip = {});

  /** Thread-safe. Drops frames if pipeline not running. */
  void pushBgrFrame(const cv::Mat& bgr);

  /** True after a successful signaling exchange built appsrc → webrtcbin. */
  bool hasMediaPipeline() const;
  /** True after at least one BGR frame supplied the real stream size. */
  bool hasFrameSize() const;
  Stats stats() const;

  std::string lastError() const;
  static std::string GstElementFromEncoderBackend(const std::string& backend_name);

  /** Hot-updated frame rate for appsrc timestamps and new WebRTC pipelines. */
  void setFps(double fps);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  Config config_;
  std::atomic<double> media_fps_{10.0};
  std::atomic<bool> running_{false};
  mutable std::mutex err_mutex_;
  std::string last_error_;

  void setError(std::string msg);
  std::string handleOfferOnGstThread(const std::string& offer_json,
                                     const std::string& client_ip);
  void scheduleLatestFramePush();
  void pushLatestFrameOnGstThread();
};

}  // namespace circle::debug
