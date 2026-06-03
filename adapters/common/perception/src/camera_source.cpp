#include "circle/perception/camera_source.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#ifdef __linux__
#include <linux/videodev2.h>
#endif

#include "circle/perception/rknn_engine.hpp"
#include "circle/types/time.hpp"

namespace circle::perception {

namespace {

constexpr int kDefaultV4L2BufferCount = 4;

#ifdef __linux__

struct V4L2CtrlCaps {
  bool supported{false};
  bool writable{false};
  int min{0};
  int max{0};
  int step{1};
  int def{0};
  uint32_t flags{0};
};

V4L2CtrlCaps queryCtrl(int fd, uint32_t id) {
  V4L2CtrlCaps caps;
  struct v4l2_queryctrl q = {};
  q.id = id;
  if (ioctl(fd, VIDIOC_QUERYCTRL, &q) != 0) {
    return caps;
  }
  if (q.flags & V4L2_CTRL_FLAG_DISABLED) {
    return caps;
  }
  caps.supported = true;
  caps.flags = q.flags;
  caps.writable = !(q.flags & V4L2_CTRL_FLAG_READ_ONLY) &&
                  !(q.flags & V4L2_CTRL_FLAG_INACTIVE);
  caps.min = q.minimum;
  caps.max = q.maximum;
  caps.step = std::max(1, q.step);
  caps.def = q.default_value;
  return caps;
}

int clampToStep(int value, const V4L2CtrlCaps& caps) {
  value = std::clamp(value, caps.min, caps.max);
  const int offset = value - caps.min;
  const int stepped = caps.min + static_cast<int>(
      std::llround(static_cast<double>(offset) / caps.step)) * caps.step;
  return std::clamp(stepped, caps.min, caps.max);
}

int pctToValue(int pct, const V4L2CtrlCaps& caps) {
  pct = std::clamp(pct, 1, 100);
  const double n = static_cast<double>(pct - 1) / 99.0;
  const int raw = caps.min + static_cast<int>(
      std::llround(n * static_cast<double>(caps.max - caps.min)));
  return clampToStep(raw, caps);
}

bool setCtrl(int fd, const char* name, uint32_t id, int32_t value,
             const char* source) {
  struct v4l2_control ctrl = {};
  ctrl.id = id;
  ctrl.value = value;
  if (ioctl(fd, VIDIOC_S_CTRL, &ctrl) == 0) {
    rknnLog(RknnLogLevel::Info, "MppCameraSource/uvc: %s = %d [%s]", name,
            value, source);
    return true;
  }
  rknnLog(RknnLogLevel::Warn, "MppCameraSource/uvc: %s = %d [%s] failed: %s",
          name, value, source, std::strerror(errno));
  return false;
}

void applyIntControl(int fd, const char* name, uint32_t id, int raw_value,
                     int pct_value, bool normalized) {
  const V4L2CtrlCaps caps = queryCtrl(fd, id);
  if (!caps.supported) {
    rknnLog(RknnLogLevel::Warn, "MppCameraSource/uvc: %s: unsupported, skipped",
            name);
    return;
  }
  if (!caps.writable) {
    rknnLog(RknnLogLevel::Warn,
            "MppCameraSource/uvc: %s: inactive/read-only (range=%d..%d step=%d "
            "def=%d), skipped",
            name, caps.min, caps.max, caps.step, caps.def);
    return;
  }
  if (normalized) {
    if (pct_value < 0) {
      return;
    }
    const int value = pctToValue(pct_value, caps);
    setCtrl(fd, name, id, value, "normalized");
    return;
  }
  if (raw_value < 0) {
    return;
  }
  const int value = clampToStep(raw_value, caps);
  if (value != raw_value) {
    rknnLog(RknnLogLevel::Warn,
            "MppCameraSource/uvc: %s raw=%d clamped to %d (range=%d..%d step=%d)",
            name, raw_value, value, caps.min, caps.max, caps.step);
  }
  setCtrl(fd, name, id, value, "raw");
}

bool setAutoExposureCtrl(int fd, bool auto_exp) {
  const int standard_value =
      auto_exp ? V4L2_EXPOSURE_AUTO : V4L2_EXPOSURE_MANUAL;
  struct v4l2_control ctrl = {};
  ctrl.id = V4L2_CID_EXPOSURE_AUTO;
  ctrl.value = standard_value;
  if (ioctl(fd, VIDIOC_S_CTRL, &ctrl) == 0) {
    rknnLog(RknnLogLevel::Info,
            "MppCameraSource/uvc: autoexposure = %d (V4L2_CID_EXPOSURE_AUTO)",
            standard_value);
    return true;
  }
  const int first_errno = errno;
  // UVC firmware sometimes reports EINVAL for the standard MANUAL/AUTO enum
  // values and only accepts the UVC raw codes (1=manual, 3=aperture priority).
  if (auto_exp && first_errno == EINVAL) {
    ctrl.value = 3;
    if (ioctl(fd, VIDIOC_S_CTRL, &ctrl) == 0) {
      rknnLog(RknnLogLevel::Info,
              "MppCameraSource/uvc: autoexposure = 3 (UVC aperture)");
      return true;
    }
  }
  rknnLog(RknnLogLevel::Warn, "MppCameraSource/uvc: autoexposure = %d failed: %s",
          standard_value, std::strerror(first_errno));
  return false;
}

void applyUvcControls(int fd, const UvcControlsConfig& cfg) {
  if (fd < 0) return;

  const bool normalized = (cfg.mode == "normalized");
  if (!normalized && cfg.mode != "raw") {
    rknnLog(RknnLogLevel::Warn,
            "MppCameraSource/uvc: unknown uvc_control_mode='%s', defaulting to raw",
            cfg.mode.c_str());
  }
  rknnLog(RknnLogLevel::Info, "MppCameraSource/uvc: applying controls (mode=%s)",
          normalized ? "normalized" : "raw");

  applyIntControl(fd, "brightness", V4L2_CID_BRIGHTNESS,
                  cfg.brightness, cfg.brightness_pct, normalized);
  applyIntControl(fd, "contrast", V4L2_CID_CONTRAST,
                  cfg.contrast, cfg.contrast_pct, normalized);
  applyIntControl(fd, "saturation", V4L2_CID_SATURATION,
                  cfg.saturation, cfg.saturation_pct, normalized);
  applyIntControl(fd, "sharpness", V4L2_CID_SHARPNESS,
                  cfg.sharpness, cfg.sharpness_pct, normalized);
  applyIntControl(fd, "gain", V4L2_CID_GAIN,
                  cfg.gain, cfg.gain_pct, normalized);
  applyIntControl(fd, "gamma", V4L2_CID_GAMMA,
                  cfg.gamma, cfg.gamma_pct, normalized);
  applyIntControl(fd, "backlight_compensation", V4L2_CID_BACKLIGHT_COMPENSATION,
                  cfg.backlight_compensation, cfg.backlight_compensation_pct,
                  normalized);

  setAutoExposureCtrl(fd, cfg.autoexposure);
  if (!cfg.autoexposure) {
    applyIntControl(fd, "exposure", V4L2_CID_EXPOSURE_ABSOLUTE,
                    cfg.exposure, cfg.exposure_pct, normalized);
  }

  // Auto-WB toggle accepts a plain 0/1 enum even when QUERYCTRL is missing.
  {
    struct v4l2_control ctrl = {};
    ctrl.id = V4L2_CID_AUTO_WHITE_BALANCE;
    ctrl.value = cfg.auto_white_balance ? 1 : 0;
    if (ioctl(fd, VIDIOC_S_CTRL, &ctrl) == 0) {
      rknnLog(RknnLogLevel::Info, "MppCameraSource/uvc: auto_white_balance = %d",
              ctrl.value);
    } else {
      rknnLog(RknnLogLevel::Warn,
              "MppCameraSource/uvc: auto_white_balance = %d failed: %s",
              ctrl.value, std::strerror(errno));
    }
  }
  if (!cfg.auto_white_balance) {
    applyIntControl(fd, "white_balance", V4L2_CID_WHITE_BALANCE_TEMPERATURE,
                    cfg.white_balance, cfg.white_balance_pct, normalized);
  }

  // Disabling autofocus silences "focus unsupported" spam from firmware that
  // exposes the control as INACTIVE; explicit enable handled via raw control
  // is rarely useful for these UVC sensors so we only honor disable.
  if (!cfg.autofocus) {
    struct v4l2_control ctrl = {};
    ctrl.id = V4L2_CID_FOCUS_AUTO;
    ctrl.value = 0;
    if (ioctl(fd, VIDIOC_S_CTRL, &ctrl) == 0) {
      rknnLog(RknnLogLevel::Info, "MppCameraSource/uvc: autofocus = 0");
    }
  }
}

#endif  // __linux__

}  // namespace

MppCameraSource::MppCameraSource(MppCameraSourceConfig config)
    : config_(std::move(config)) {}

bool MppCameraSource::start() {
  if (running_) {
    return true;
  }
  if (config_.width == 0) {
    config_.width = 1280;
  }
  if (config_.height == 0) {
    config_.height = 1024;
  }
  if (config_.fps == 0) {
    config_.fps = 60;
  }
  if (config_.output_width == 0) {
    config_.output_width = 640;
  }
  if (config_.output_height == 0) {
    config_.output_height = 512;
  }

#if CIRCLE_PERCEPTION_USE_MPP_RGA
  if (!pipeline_.init(static_cast<int>(config_.width),
                      static_cast<int>(config_.height),
                      static_cast<int>(config_.output_width),
                      static_cast<int>(config_.output_height))) {
    std::cerr << "MppCameraSource: MppRgaPipeline init failed\n";
  } else {
    rgb_scratch_.resize(static_cast<size_t>(pipeline_.outputWidth()) *
                        static_cast<size_t>(pipeline_.outputHeight()) * 3u);
  }
#else
  std::cerr << "MppCameraSource: MPP/RGA not available; MJPEG-only grab\n";
#endif

#ifdef __linux__
  if (!openV4L2()) {
    return false;
  }
#else
  std::cerr << "MppCameraSource: non-linux platform, skipping real camera init\n";
#endif
  running_ = true;
  return true;
}

void MppCameraSource::stop() {
  closeV4L2();
#if CIRCLE_PERCEPTION_USE_MPP_RGA
  pipeline_.destroy();
  rgb_scratch_.clear();
#endif
  running_ = false;
}

bool MppCameraSource::grab(FrameReady& out) {
  if (!running_) {
    return false;
  }

#ifdef __linux__
  if (active_buffer_index_ >= 0) {
    struct v4l2_buffer qbuf = {};
    qbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    qbuf.memory = V4L2_MEMORY_MMAP;
    qbuf.index = static_cast<uint32_t>(active_buffer_index_);
    if (ioctl(v4l2_fd_, VIDIOC_QBUF, &qbuf) < 0) {
      std::cerr << "MppCameraSource: VIDIOC_QBUF index=" << active_buffer_index_
                << " failed: " << std::strerror(errno) << '\n';
    }
    active_buffer_index_ = -1;
  }
#endif

  out.seq = ++seq_;
  out.capture_ns = circle::types::monotonicNowNs();
  out.data = nullptr;
  out.data_size = 0;
  out.hw_rgb = false;
  out.fd = -1;
  out.capture_time_ms = 0.0F;
  out.decode_time_ms = 0.0F;

#ifdef __linux__
  CapturedFrame frame;
  const auto capture_start = std::chrono::steady_clock::now();
  if (!captureMjpegFrame(frame)) {
    return false;
  }
  const auto capture_end = std::chrono::steady_clock::now();
  out.capture_time_ms =
      std::chrono::duration<float, std::milli>(capture_end - capture_start).count();
  active_buffer_index_ = static_cast<int32_t>(frame.index);
#if CIRCLE_PERCEPTION_USE_MPP_RGA
  if (pipeline_.isInitialized() && !rgb_scratch_.empty()) {
    const auto decode_start = std::chrono::steady_clock::now();
    if (decodeMjpegToRgb(static_cast<const uint8_t*>(frame.data), frame.size, out)) {
      const auto decode_end = std::chrono::steady_clock::now();
      out.decode_time_ms =
          std::chrono::duration<float, std::milli>(decode_end - decode_start).count();
      return true;
    }
  }
#endif
  out.width = config_.width;
  out.height = config_.height;
  out.data = static_cast<const uint8_t*>(frame.data);
  out.data_size = frame.size;
#else
  out.width = config_.width;
  out.height = config_.height;
#endif
  return true;
}

bool MppCameraSource::decodeMjpegToRgb(const uint8_t* mjpeg, size_t mjpeg_size,
                                       FrameReady& out) {
#if !CIRCLE_PERCEPTION_USE_MPP_RGA
  (void)mjpeg;
  (void)mjpeg_size;
  (void)out;
  return false;
#else
  if (!mjpeg || mjpeg_size == 0 || rgb_scratch_.empty()) {
    return false;
  }
  int out_w = 0;
  int out_h = 0;
  if (!pipeline_.decodeAndConvert(mjpeg, mjpeg_size, rgb_scratch_.data(), &out_w, &out_h)) {
    return false;
  }
  out.width = static_cast<uint32_t>(out_w);
  out.height = static_cast<uint32_t>(out_h);
  out.data = rgb_scratch_.data();
  out.data_size = rgb_scratch_.size();
  out.hw_rgb = true;
  return true;
#endif
}

#ifdef __linux__

bool MppCameraSource::openV4L2() {
  v4l2_fd_ = open(config_.device.c_str(), O_RDWR | O_NONBLOCK);
  if (v4l2_fd_ < 0) {
    std::cerr << "MppCameraSource: open(" << config_.device << ") failed: "
              << std::strerror(errno) << '\n';
    return false;
  }

  struct v4l2_format fmt = {};
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = config_.width;
  fmt.fmt.pix.height = config_.height;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
  if (ioctl(v4l2_fd_, VIDIOC_S_FMT, &fmt) < 0) {
    std::cerr << "MppCameraSource: VIDIOC_S_FMT MJPEG failed: "
              << std::strerror(errno) << '\n';
    close(v4l2_fd_);
    v4l2_fd_ = -1;
    return false;
  }

  config_.width = static_cast<uint32_t>(fmt.fmt.pix.width);
  config_.height = static_cast<uint32_t>(fmt.fmt.pix.height);

  struct v4l2_streamparm parm = {};
  parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  parm.parm.capture.timeperframe.numerator = 1;
  parm.parm.capture.timeperframe.denominator =
      static_cast<uint32_t>(config_.fps > 0 ? config_.fps : 30);
  if (ioctl(v4l2_fd_, VIDIOC_S_PARM, &parm) < 0) {
    std::cerr << "MppCameraSource: VIDIOC_S_PARM failed: "
              << std::strerror(errno) << '\n';
  }

  // Lock down image quality before streaming so AE/AWB don't drift between
  // frames and YOLO confidence stays stable.
  applyUvcControls(v4l2_fd_, config_.uvc);

  struct v4l2_requestbuffers req = {};
  req.count = kDefaultV4L2BufferCount;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
  if (ioctl(v4l2_fd_, VIDIOC_REQBUFS, &req) < 0) {
    std::cerr << "MppCameraSource: VIDIOC_REQBUFS failed: "
              << std::strerror(errno) << '\n';
    close(v4l2_fd_);
    v4l2_fd_ = -1;
    return false;
  }

  v4l2_buffer_count_ = static_cast<int>(req.count);
  v4l2_buffers_.clear();
  v4l2_buffer_sizes_.clear();
  for (int i = 0; i < v4l2_buffer_count_; ++i) {
    struct v4l2_buffer buf = {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = static_cast<uint32_t>(i);
    if (ioctl(v4l2_fd_, VIDIOC_QUERYBUF, &buf) < 0) {
      std::cerr << "MppCameraSource: VIDIOC_QUERYBUF index=" << i
                << " failed: " << std::strerror(errno) << '\n';
      closeV4L2();
      return false;
    }

    void* ptr = mmap(nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED,
                     v4l2_fd_, buf.m.offset);
    if (ptr == MAP_FAILED) {
      std::cerr << "MppCameraSource: mmap index=" << i << " failed: "
                << std::strerror(errno) << '\n';
      closeV4L2();
      return false;
    }

    v4l2_buffers_.push_back(ptr);
    v4l2_buffer_sizes_.push_back(static_cast<size_t>(buf.length));
  }

  for (int i = 0; i < v4l2_buffer_count_; ++i) {
    struct v4l2_buffer buf = {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = static_cast<uint32_t>(i);
    if (ioctl(v4l2_fd_, VIDIOC_QBUF, &buf) < 0) {
      std::cerr << "MppCameraSource: VIDIOC_QBUF index=" << i
                << " failed: " << std::strerror(errno) << '\n';
      closeV4L2();
      return false;
    }
  }

  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(v4l2_fd_, VIDIOC_STREAMON, &type) < 0) {
    std::cerr << "MppCameraSource: VIDIOC_STREAMON failed: "
              << std::strerror(errno) << '\n';
    closeV4L2();
    return false;
  }
  v4l2_streaming_ = true;
  return true;
}

void MppCameraSource::closeV4L2() {
  if (v4l2_fd_ < 0) {
    return;
  }

  active_buffer_index_ = -1;

  if (v4l2_streaming_) {
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(v4l2_fd_, VIDIOC_STREAMOFF, &type);
    v4l2_streaming_ = false;
  }

  for (size_t i = 0; i < v4l2_buffers_.size(); ++i) {
    if (v4l2_buffers_[i] && i < v4l2_buffer_sizes_.size()) {
      munmap(v4l2_buffers_[i], v4l2_buffer_sizes_[i]);
    }
  }
  v4l2_buffers_.clear();
  v4l2_buffer_sizes_.clear();
  close(v4l2_fd_);
  v4l2_fd_ = -1;
  v4l2_buffer_count_ = 0;
}

bool MppCameraSource::captureMjpegFrame(CapturedFrame& out_frame) {
  if (v4l2_fd_ < 0) {
    return false;
  }

  struct v4l2_buffer buf = {};
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;
  if (ioctl(v4l2_fd_, VIDIOC_DQBUF, &buf) < 0) {
    if (errno == EAGAIN) {
      struct pollfd pfd{};
      pfd.fd = v4l2_fd_;
      pfd.events = POLLIN;
      if (poll(&pfd, 1, 2) <= 0) {
        return false;
      }
      if (ioctl(v4l2_fd_, VIDIOC_DQBUF, &buf) < 0) {
        return false;
      }
    } else {
      return false;
    }
  }

  if (buf.index >= static_cast<uint32_t>(v4l2_buffers_.size())) {
    return false;
  }

  if (buf.index >= v4l2_buffer_sizes_.size()) {
    return false;
  }
  if (buf.bytesused == 0 || buf.bytesused > v4l2_buffer_sizes_[buf.index]) {
    std::cerr << "MppCameraSource: invalid bytesused=" << buf.bytesused
              << " for index=" << buf.index << '\n';
    return false;
  }

  out_frame.data = v4l2_buffers_[buf.index];
  out_frame.size = static_cast<size_t>(buf.bytesused);
  out_frame.index = buf.index;

  return true;
}

#endif  // __linux__

}  // namespace circle::perception
