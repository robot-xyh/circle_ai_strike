#include "circle/perception/zero_copy_perception_runtime.hpp"

#include <cerrno>
#include <algorithm>
#include <chrono>
#include <cstring>
#include <iostream>
#include <poll.h>

#ifdef __linux__
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <linux/videodev2.h>
#endif

#include "circle/types/time.hpp"

namespace circle::perception {
namespace {
#if CIRCLE_PERCEPTION_USE_MPP_RGA && CIRCLE_PERCEPTION_USE_RKNN
int64_t steadyToNs(std::chrono::steady_clock::time_point tp) {
  return static_cast<int64_t>(
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          tp.time_since_epoch())
          .count());
}
#endif
}  // namespace


namespace {

constexpr int kDefaultV4L2BufferCount = 4;

#if CIRCLE_PERCEPTION_USE_MPP_RGA && CIRCLE_PERCEPTION_USE_RKNN
ZeroCopySlotGeometry computeGeometry(int out_w, int out_h, int model_w, int model_h) {
  ZeroCopySlotGeometry g;
  g.model_w = static_cast<uint32_t>(model_w);
  g.model_h = static_cast<uint32_t>(model_h);
  g.content_w = out_w;
  g.content_h = out_h;
  const double scale = std::min(
      static_cast<double>(model_w) / std::max(1, out_w),
      static_cast<double>(model_h) / std::max(1, out_h));
  const int content_w = static_cast<int>(std::round(out_w * scale));
  const int content_h = static_cast<int>(std::round(out_h * scale));
  const int left_pad = static_cast<int>(
      std::round((static_cast<int>(model_w) - content_w) / 2.0 - 0.1));
  const int top_pad = static_cast<int>(
      std::round((static_cast<int>(model_h) - content_h) / 2.0 - 0.1));
  g.content_w = content_w;
  g.content_h = content_h;
  g.byte_offset =
      static_cast<uint32_t>((static_cast<size_t>(top_pad) * model_w + left_pad) * 3);
  return g;
}
#endif

}  // namespace

ZeroCopyPerceptionRuntime::ZeroCopyPerceptionRuntime(ZeroCopyPerceptionConfig config)
    : config_(std::move(config)),
      vision_(config_.vision) {
  config_.slot_count = std::clamp(config_.slot_count, 1, 4);
  config_.infer_worker_count =
      std::clamp(config_.infer_worker_count, 1, config_.slot_count);
}

ZeroCopyPerceptionRuntime::~ZeroCopyPerceptionRuntime() { stop(); }

bool ZeroCopyPerceptionRuntime::initialize() {
  if (config_.camera.width == 0) {
    config_.camera.width = 1280;
  }
  if (config_.camera.height == 0) {
    config_.camera.height = 1024;
  }
  if (config_.camera.fps == 0) {
    config_.camera.fps = 60;
  }
  if (config_.camera.output_width == 0) {
    config_.camera.output_width = 640;
  }
  if (config_.camera.output_height == 0) {
    config_.camera.output_height = 512;
  }

#if !CIRCLE_PERCEPTION_USE_MPP_RGA || !CIRCLE_PERCEPTION_USE_RKNN
  return false;
#else
  if (!vision_.initialize(config_.slot_count)) {
    return false;
  }
  if (!pipeline_.init(static_cast<int>(config_.camera.width),
                      static_cast<int>(config_.camera.height),
                      static_cast<int>(config_.camera.output_width),
                      static_cast<int>(config_.camera.output_height))) {
    return false;
  }

  geometry_ = computeGeometry(pipeline_.outputWidth(), pipeline_.outputHeight(),
                              config_.vision.letterbox_w,
                              config_.vision.letterbox_h);
  const uint32_t buffer_size = vision_.inputBufferSize();
  if (buffer_size == 0) {
    return false;
  }
  if (geometry_.content_w != pipeline_.outputWidth() ||
      geometry_.content_h != pipeline_.outputHeight()) {
    std::cerr << "ZeroCopyPerceptionRuntime: letterbox content must match pipeline output\n";
    return false;
  }
  if (geometry_.byte_offset +
          static_cast<uint32_t>(static_cast<size_t>(pipeline_.outputWidth()) *
                                pipeline_.outputHeight() * 3) > buffer_size) {
    std::cerr << "ZeroCopyPerceptionRuntime: RKNN input buffer too small\n";
    return false;
  }
  geometry_.buffer_size = buffer_size;

  std::vector<int> dma_fds;
  dma_fds.reserve(static_cast<size_t>(config_.slot_count));
  for (int i = 0; i < config_.slot_count; ++i) {
    const int fd = vision_.slotDmaFd(i);
    if (fd < 0) {
      return false;
    }
    dma_fds.push_back(fd);
  }

  std::string error;
  if (!configureZeroCopySlots(
          pipeline_, dma_fds, buffer_size, geometry_.content_w, geometry_.content_h,
          static_cast<int>(geometry_.model_w), static_cast<int>(geometry_.model_h),
          geometry_.byte_offset, &slots_, &error)) {
    std::cerr << "ZeroCopyPerceptionRuntime: " << error << '\n';
    return false;
  }

  for (int i = 0; i < config_.slot_count; ++i) {
    auto* input = vision_.engine(i).getInputBuffer();
    const uint32_t size = vision_.engine(i).getInputBufferSize();
    if (input && size > 0) {
      std::memset(input, 114, size);
    }
  }

  return openV4L2();
#endif
}

bool ZeroCopyPerceptionRuntime::start(ResultCallback on_result,
                                      std::atomic<bool>* running) {
  if (!on_result || !running) {
    return false;
  }
  on_result_ = std::move(on_result);
  running_ = running;

  producer_thread_ = std::thread(&ZeroCopyPerceptionRuntime::producerLoop, this);
  infer_threads_.clear();
  infer_threads_.reserve(static_cast<size_t>(config_.infer_worker_count));
  for (int i = 0; i < config_.infer_worker_count; ++i) {
    infer_threads_.emplace_back(&ZeroCopyPerceptionRuntime::inferWorkerLoop, this);
  }
  return true;
}

void ZeroCopyPerceptionRuntime::stop() {
  if (running_) {
    running_->store(false);
  }
  ready_cv_.notify_all();
  if (producer_thread_.joinable()) {
    producer_thread_.join();
  }
  for (auto& t : infer_threads_) {
    if (t.joinable()) {
      t.join();
    }
  }
  infer_threads_.clear();
  closeV4L2();
  releaseZeroCopySlots(&slots_, geometry_.buffer_size);
  pipeline_.destroy();
}

#ifdef __linux__

bool ZeroCopyPerceptionRuntime::openV4L2() {
  v4l2_fd_ = open(config_.camera.device.c_str(), O_RDWR | O_NONBLOCK);
  if (v4l2_fd_ < 0) {
    return false;
  }

  struct v4l2_format fmt = {};
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = config_.camera.width;
  fmt.fmt.pix.height = config_.camera.height;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
  if (ioctl(v4l2_fd_, VIDIOC_S_FMT, &fmt) < 0) {
    closeV4L2();
    return false;
  }

  struct v4l2_streamparm parm = {};
  parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  parm.parm.capture.timeperframe.numerator = 1;
  parm.parm.capture.timeperframe.denominator = config_.camera.fps;
  (void)ioctl(v4l2_fd_, VIDIOC_S_PARM, &parm);

  struct v4l2_requestbuffers req = {};
  req.count = kDefaultV4L2BufferCount;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
  if (ioctl(v4l2_fd_, VIDIOC_REQBUFS, &req) < 0) {
    closeV4L2();
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
      closeV4L2();
      return false;
    }
    void* ptr = mmap(nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED,
                     v4l2_fd_, buf.m.offset);
    if (ptr == MAP_FAILED) {
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
      closeV4L2();
      return false;
    }
  }

  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(v4l2_fd_, VIDIOC_STREAMON, &type) < 0) {
    closeV4L2();
    return false;
  }
  v4l2_streaming_ = true;
  return true;
}

void ZeroCopyPerceptionRuntime::closeV4L2() {
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

bool ZeroCopyPerceptionRuntime::captureMjpeg(void** buffer, size_t* size,
                                             int* buf_index) {
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
  if (buf.index >= v4l2_buffers_.size() || buf.bytesused == 0) {
    return false;
  }
  *buffer = v4l2_buffers_[buf.index];
  *size = static_cast<size_t>(buf.bytesused);
  *buf_index = static_cast<int>(buf.index);
  return true;
}

void ZeroCopyPerceptionRuntime::requeueV4L2Buffer(int index) {
  if (v4l2_fd_ < 0 || index < 0) {
    return;
  }
  struct v4l2_buffer buf = {};
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;
  buf.index = static_cast<uint32_t>(index);
  (void)ioctl(v4l2_fd_, VIDIOC_QBUF, &buf);
}

#else

bool ZeroCopyPerceptionRuntime::openV4L2() { return false; }
void ZeroCopyPerceptionRuntime::closeV4L2() {}
bool ZeroCopyPerceptionRuntime::captureMjpeg(void**, size_t*, int*) { return false; }
void ZeroCopyPerceptionRuntime::requeueV4L2Buffer(int) {}

#endif

void ZeroCopyPerceptionRuntime::copyPreviewBgrFromSlot(uint32_t slot_id,
                                                       std::vector<uint8_t>* bgr,
                                                       uint32_t width,
                                                       uint32_t height) {
  if (!bgr || slot_id >= slots_.size() || !slots_[slot_id].mmap_ptr) {
    return;
  }
  const uint32_t stride = width * 3u;
  const size_t bytes = static_cast<size_t>(stride) * height;
  if (bytes == 0) {
    return;
  }
  bgr->resize(bytes);
  const size_t src_row_stride = static_cast<size_t>(geometry_.model_w) * 3u;
  const size_t src_offset = geometry_.byte_offset;
  syncDmaBufForCpuRead(slots_[slot_id].dma_fd, true);
  const auto* src_base =
      static_cast<const uint8_t*>(slots_[slot_id].mmap_ptr) + src_offset;
  for (uint32_t y = 0; y < height; ++y) {
    const uint8_t* rgb = src_base + static_cast<size_t>(y) * src_row_stride;
    uint8_t* dst = bgr->data() + static_cast<size_t>(y) * stride;
    for (uint32_t x = 0; x < width; ++x) {
      dst[x * 3 + 0] = rgb[x * 3 + 2];
      dst[x * 3 + 1] = rgb[x * 3 + 1];
      dst[x * 3 + 2] = rgb[x * 3 + 0];
    }
  }
  syncDmaBufForCpuRead(slots_[slot_id].dma_fd, false);
}

ZeroCopyRuntimeStats ZeroCopyPerceptionRuntime::stats() const {
  ZeroCopyRuntimeStats out;
  out.grab_ok = grab_ok_;
  out.infer_det = infer_det_;
  out.infer_no_det = infer_no_det_;
  out.infer_rknn_fail = infer_rknn_fail_;
  out.slot_busy_drops = slot_busy_drops_;
  out.ready_drops = ready_drops_;
  return out;
}

void ZeroCopyPerceptionRuntime::producerLoop() {
#if CIRCLE_PERCEPTION_USE_MPP_RGA && CIRCLE_PERCEPTION_USE_RKNN
  const auto period = std::chrono::microseconds(
      config_.camera.fps > 0
          ? static_cast<int64_t>(1'000'000 / config_.camera.fps)
          : 16'667);
  auto next = std::chrono::steady_clock::now();
  auto last_log = next;

  while (running_ && running_->load()) {
    int slot_id = -1;
    {
      std::lock_guard<std::mutex> lk(pipeline_mu_);
      slot_id = leaseFreeZeroCopySlot(&slots_);
    }
    if (slot_id < 0) {
      ++slot_busy_drops_;
      std::this_thread::sleep_for(std::chrono::microseconds(500));
      continue;
    }

    void* mjpeg = nullptr;
    size_t mjpeg_size = 0;
    int buf_index = -1;
    const auto grab_start = std::chrono::steady_clock::now();
    if (!captureMjpeg(&mjpeg, &mjpeg_size, &buf_index)) {
      {
        std::lock_guard<std::mutex> lk(pipeline_mu_);
        releaseZeroCopySlotLease(&slots_, static_cast<uint32_t>(slot_id));
      }
      ++grab_fail_;
      next += period;
      std::this_thread::sleep_until(next);
      continue;
    }

    const auto grab_done = std::chrono::steady_clock::now();
    int out_w = 0;
    int out_h = 0;
    bool ok = false;
    {
      std::lock_guard<std::mutex> lk(pipeline_mu_);
      pipeline_.switchExternalDstHandle(slots_[static_cast<size_t>(slot_id)].rga_handle,
                                        slots_[static_cast<size_t>(slot_id)].mmap_ptr);
      if (geometry_.byte_offset > 0) {
        syncDmaBufForCpuWrite(slots_[static_cast<size_t>(slot_id)].dma_fd, true);
      }
      ok = pipeline_.decodeAndConvert(static_cast<const uint8_t*>(mjpeg), mjpeg_size,
                                      nullptr, &out_w, &out_h);
      if (geometry_.byte_offset > 0) {
        syncDmaBufForCpuWrite(slots_[static_cast<size_t>(slot_id)].dma_fd, false);
      }
    }
    requeueV4L2Buffer(buf_index);

    const auto producer_done = std::chrono::steady_clock::now();
    if (!ok) {
      std::lock_guard<std::mutex> lk(pipeline_mu_);
      releaseZeroCopySlotLease(&slots_, static_cast<uint32_t>(slot_id));
      ++grab_fail_;
      next += period;
      std::this_thread::sleep_until(next);
      continue;
    }

    ++grab_ok_;
    ReadyFrame ready;
    ready.slot_id = static_cast<uint32_t>(slot_id);
    ready.seq = ++seq_;
    ready.capture_ns = static_cast<int64_t>(circle::types::monotonicNowNs());
    ready.grab_start = grab_start;
    ready.grab_done = grab_done;
    ready.producer_done = producer_done;
    ready.capture_time_ms =
        std::chrono::duration<float, std::milli>(grab_done - grab_start).count();
    ready.decode_time_ms =
        std::chrono::duration<float, std::milli>(producer_done - grab_done).count();
    ready.src_w = out_w;
    ready.src_h = out_h;
    slots_[static_cast<size_t>(slot_id)].seq =
        static_cast<uint32_t>(ready.seq);

    {
      std::lock_guard<std::mutex> lk(ready_mu_);
      if (ready_queue_.size() >= kMaxReadyQueue) {
        const ReadyFrame dropped = ready_queue_.front();
        ready_queue_.pop_front();
        ++ready_drops_;
        std::lock_guard<std::mutex> plk(pipeline_mu_);
        releaseZeroCopySlotLease(&slots_, dropped.slot_id);
      }
      ready_queue_.push_back(ready);
    }
    ready_cv_.notify_one();

    const auto now = std::chrono::steady_clock::now();
    if (now - last_log >= std::chrono::seconds(5)) {
      std::cerr << "zero_copy/camera: grab_ok=" << grab_ok_
                << " grab_fail=" << grab_fail_
                << " slot_busy_drops=" << slot_busy_drops_
                << " ready_drops=" << ready_drops_
                << " slots=" << config_.slot_count << '\n';
      last_log = now;
    }

    next += period;
    std::this_thread::sleep_until(next);
  }
#endif
}

void ZeroCopyPerceptionRuntime::inferWorkerLoop() {
#if CIRCLE_PERCEPTION_USE_RKNN
  auto last_infer = std::chrono::steady_clock::time_point{};
  auto last_log = std::chrono::steady_clock::now();

  while (running_ && running_->load()) {
    ReadyFrame ready;
    {
      std::unique_lock<std::mutex> lk(ready_mu_);
      ready_cv_.wait_for(lk, std::chrono::milliseconds(50), [&]() {
        return !running_->load() || !ready_queue_.empty();
      });
      if (!running_->load()) {
        break;
      }
      if (ready_queue_.empty()) {
        continue;
      }
      ready = ready_queue_.front();
      ready_queue_.pop_front();
    }

    const auto infer_start = std::chrono::steady_clock::now();

    if (config_.max_infer_fps > 0) {
      const auto min_dt = std::chrono::duration<double>(1.0 / config_.max_infer_fps);
      const auto now = std::chrono::steady_clock::now();
      if (last_infer.time_since_epoch().count() > 0 && now - last_infer < min_dt) {
        std::lock_guard<std::mutex> plk(pipeline_mu_);
        releaseZeroCopySlotLease(&slots_, ready.slot_id);
        continue;
      }
      last_infer = std::chrono::steady_clock::now();
    }

    circle::types::FrameDetection det;
    VisionPipelineTiming timing;
    const InferSlotResult infer_result = vision_.inferPreparedSlot(
        static_cast<int>(ready.slot_id), ready.src_w, ready.src_h, ready.seq,
        static_cast<circle::types::TimestampNs>(ready.capture_ns), det, &timing);
    const bool valid = infer_result == InferSlotResult::Detection;

    ZeroCopyInferResult result;
    result.detection = det;
    result.timing = timing;
    result.frame_seq = ready.seq;
    result.stamp_ns = ready.capture_ns;
    const auto infer_done = std::chrono::steady_clock::now();
    result.capture_time_ms = ready.capture_time_ms;
    result.decode_time_ms = ready.decode_time_ms;
    result.pipeline.wait_grab_ms = ready.capture_time_ms;
    result.pipeline.producer_ms = ready.decode_time_ms;
    result.pipeline.queue_wait_ms =
        std::chrono::duration<float, std::milli>(infer_start - ready.producer_done)
            .count();
    result.pipeline.cnn_ms =
        std::chrono::duration<float, std::milli>(infer_done - infer_start).count();
    result.pipeline.grab_start_steady_ns = steadyToNs(ready.grab_start);
    result.pipeline.grab_done_steady_ns = steadyToNs(ready.grab_done);
    result.pipeline.producer_done_steady_ns = steadyToNs(ready.producer_done);
    result.pipeline.infer_start_steady_ns = steadyToNs(infer_start);
    result.pipeline.infer_done_steady_ns = steadyToNs(infer_done);
    result.pipeline.grab_done_ns = result.pipeline.grab_done_steady_ns;
    result.preview_w = static_cast<uint32_t>(ready.src_w);
    result.preview_h = static_cast<uint32_t>(ready.src_h);
    if (valid || true) {
      copyPreviewBgrFromSlot(ready.slot_id, &result.preview_bgr, result.preview_w,
                             result.preview_h);
    }

    {
      std::lock_guard<std::mutex> plk(pipeline_mu_);
      releaseZeroCopySlotLease(&slots_, ready.slot_id);
    }

    switch (infer_result) {
      case InferSlotResult::Detection:
        ++infer_det_;
        break;
      case InferSlotResult::NoDetection:
        ++infer_no_det_;
        break;
      case InferSlotResult::RknnFailed:
        ++infer_rknn_fail_;
        break;
    }
    if (on_result_) {
      on_result_(std::move(result));
    }

    const auto now = std::chrono::steady_clock::now();
    if (now - last_log >= std::chrono::seconds(5)) {
      std::cerr << "zero_copy/infer: det=" << infer_det_
                << " no_det=" << infer_no_det_
                << " rknn_fail=" << infer_rknn_fail_
                << " workers=" << config_.infer_worker_count << '\n';
      last_log = now;
    }
  }
#endif
}

}  // namespace circle::perception
