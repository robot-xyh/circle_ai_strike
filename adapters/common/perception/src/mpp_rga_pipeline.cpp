/**
 * @file mpp_rga_pipeline.cpp
 * @brief MJPEG -> MPP (NV12 DMA buf) -> RGA (RGB DMA buf). Built only when USE_MPP_RGA=1.
 *
 * MJPEG on Rockchip MPP 1.5.0 (BSP) requires the "advanced" (task-based) API.
 * The "simple" API (decode_put_packet / decode_get_frame) returns
 * MPP_ERR_BUFFER_FULL (-1012) for MJPEG — confirmed by the official
 * mpi_dec_test.c which sets  cmd->simple = (type != MJPEG).
 *
 * Advanced decode flow per frame:
 *   1. Copy MJPEG bytes into pre-allocated input MppBuffer
 *   2. mpp_packet_init_with_buffer(&pkt, input_buf)
 *   3. poll(INPUT) -> dequeue(INPUT) -> set_packet+set_frame -> enqueue(INPUT)
 *   4. poll(OUTPUT) -> dequeue(OUTPUT) -> get_frame -> process -> enqueue(OUTPUT)
 *   5. Reclaim input task and release packet
 */

#include "circle/perception/mpp_rga_pipeline.hpp"

#include <array>
#include <cstdarg>
#include <cstdio>
#include <string>
/* Debug logging: set to 1 to enable per-frame MPP/RGA logs */
#ifndef MPP_RGA_DEBUG_LOG
#define MPP_RGA_DEBUG_LOG 0
#endif
#if MPP_RGA_DEBUG_LOG
#define MLOG(...) \
  do { \
    if (circle::perception::mppRgaLogEnabled( \
            circle::perception::MppRgaLogLevel::Debug)) { \
      circle::perception::mppRgaLog( \
          circle::perception::MppRgaLogLevel::Debug, __VA_ARGS__); \
    } \
  } while (0)
#else
#define MLOG(...) do {} while(0)
#endif
#define MLOG_DEBUG(...) \
  do { \
    if (circle::perception::mppRgaLogEnabled( \
            circle::perception::MppRgaLogLevel::Debug)) { \
      circle::perception::mppRgaLog( \
          circle::perception::MppRgaLogLevel::Debug, __VA_ARGS__); \
    } \
  } while (0)
#define MLOG_INFO(...) \
  do { \
    if (circle::perception::mppRgaLogEnabled( \
            circle::perception::MppRgaLogLevel::Info)) { \
      circle::perception::mppRgaLog( \
          circle::perception::MppRgaLogLevel::Info, __VA_ARGS__); \
    } \
  } while (0)
#define MLOG_WARN(...) \
  do { \
    if (circle::perception::mppRgaLogEnabled( \
            circle::perception::MppRgaLogLevel::Warn)) { \
      circle::perception::mppRgaLog( \
          circle::perception::MppRgaLogLevel::Warn, __VA_ARGS__); \
    } \
  } while (0)
#define MLOG_ERR(...) \
  do { \
    if (circle::perception::mppRgaLogEnabled( \
            circle::perception::MppRgaLogLevel::Error)) { \
      circle::perception::mppRgaLog( \
          circle::perception::MppRgaLogLevel::Error, __VA_ARGS__); \
    } \
  } while (0)

namespace circle::perception {

namespace {

MppRgaLogEnabledHandler& enabledHandlerStorage() {
  static MppRgaLogEnabledHandler handler = nullptr;
  return handler;
}

MppRgaLogHandler& logHandlerStorage() {
  static MppRgaLogHandler handler = nullptr;
  return handler;
}

bool defaultMppRgaLogEnabled(MppRgaLogLevel level) {
  return level != MppRgaLogLevel::Debug;
}

void defaultMppRgaLog(MppRgaLogLevel, const char* message) {
  std::fprintf(stderr, "%s\n", message);
  std::fflush(stderr);
}

}  // namespace

void setMppRgaLogHandler(MppRgaLogEnabledHandler enabled_handler,
                         MppRgaLogHandler log_handler) {
  enabledHandlerStorage() = enabled_handler;
  logHandlerStorage() = log_handler;
}

bool mppRgaLogEnabled(MppRgaLogLevel level) {
  MppRgaLogEnabledHandler handler = enabledHandlerStorage();
  if (handler) {
    return handler(level);
  }
  return defaultMppRgaLogEnabled(level);
}

void mppRgaLog(MppRgaLogLevel level, const char* fmt, ...) {
  std::array<char, 1024> buf{};
  va_list args;
  va_start(args, fmt);
  const int n = std::vsnprintf(buf.data(), buf.size(), fmt, args);
  va_end(args);
  if (n < 0) {
    return;
  }
  if (static_cast<size_t>(n) >= buf.size()) {
    buf.back() = '\0';
  }
  std::string message(buf.data());
  while (!message.empty() &&
         (message.back() == '\n' || message.back() == '\r')) {
    message.pop_back();
  }
  MppRgaLogHandler handler = logHandlerStorage();
  if (handler) {
    handler(level, message.c_str());
  } else {
    defaultMppRgaLog(level, message.c_str());
  }
}

}  // namespace circle::perception

#if !USE_MPP_RGA
namespace circle::perception {

MppRgaPipeline::~MppRgaPipeline() { destroy(); }
bool MppRgaPipeline::init(int, int, int, int) { return false; }
void MppRgaPipeline::destroy() { initialized_ = false; }
bool MppRgaPipeline::decodeAndConvert(
    const uint8_t*, size_t, uint8_t*, int*, int*) { return false; }
bool MppRgaPipeline::setExternalDstDma(int, uint32_t, void*, uint32_t, int, int, int, int) { return false; }
void MppRgaPipeline::switchExternalDstHandle(int, void*) {}
int MppRgaPipeline::importExternalBuffer(int, uint32_t) { return -1; }
void MppRgaPipeline::releaseExternalBuffer(int) {}

}  // namespace circle::perception

#else  // USE_MPP_RGA

#include <cstring>
#include <unistd.h>

namespace {

void copyPackedRgb(uint8_t* dst, const uint8_t* src, int width, int height,
                   int src_row_stride_pixels) {
  if (!dst || !src || width <= 0 || height <= 0) {
    return;
  }
  const size_t row_bytes = static_cast<size_t>(width) * 3u;
  const size_t src_step =
      static_cast<size_t>(src_row_stride_pixels > 0 ? src_row_stride_pixels
                                                    : width) *
      3u;
  for (int row = 0; row < height; ++row) {
    std::memcpy(dst + static_cast<size_t>(row) * row_bytes,
                src + static_cast<size_t>(row) * src_step, row_bytes);
  }
}

}  // namespace

#if __has_include(<rockchip/rk_mpi.h>)
  #include <rockchip/rk_mpi.h>
  #include <rockchip/mpp_packet.h>
  #include <rockchip/mpp_frame.h>
  #include <rockchip/mpp_buffer.h>
  #include <rockchip/mpp_meta.h>
  #include <rockchip/mpp_task.h>
#elif __has_include(<rk_mpi.h>)
  #include <rk_mpi.h>
  #include <mpp_packet.h>
  #include <mpp_frame.h>
  #include <mpp_buffer.h>
  #include <mpp_meta.h>
  #include <mpp_task.h>
#else
  #include <rk_mpi.h>
  #include <mpp_packet.h>
  #include <mpp_frame.h>
  #include <mpp_buffer.h>
  #include <mpp_meta.h>
  #include <mpp_task.h>
#endif

#if __has_include(<im2d.h>)
  #include <im2d.h>
  #include <rga.h>
  #define HAVE_RGA_IM2D 1
#elif __has_include(<rga/im2d.h>)
  #include <rga/im2d.h>
  #include <rga/rga.h>
  #define HAVE_RGA_IM2D 1
#else
  #include <RgaUtils.h>
  #include <rga.h>
  #define HAVE_RGA_IM2D 0
#endif

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

namespace circle::perception {

MppRgaPipeline::~MppRgaPipeline() { destroy(); }

// ─────────────────────────────────────────────────────────────────────────────
bool MppRgaPipeline::init(int max_width, int max_height, int out_width, int out_height)
{
  if (initialized_) return true;
  max_width_  = max_width;
  max_height_ = max_height;

  // Determine effective output dimensions
  resize_enabled_ = (out_width > 0 && out_height > 0 &&
                     (out_width != max_width || out_height != max_height));
  if (resize_enabled_) {
    out_width_  = out_width;
    out_height_ = out_height;
  } else {
    out_width_  = max_width;
    out_height_ = max_height;
  }
  int eff_w = out_width_;
  int eff_h = out_height_;

  // ── 1. Create MPP context ──────────────────────────────────────────────
  MppCtx  ctx = nullptr;
  MppApi* api = nullptr;
  if (mpp_create(&ctx, &api) != MPP_OK || !ctx || !api) {
    MLOG_ERR("[mpp_cam] ERROR: mpp_create failed — MPP hardware decoder unavailable\n");
    return false;
  }
  if (mpp_init(ctx, MPP_CTX_DEC, MPP_VIDEO_CodingMJPEG) != MPP_OK) {
    MLOG_ERR("[mpp_cam] ERROR: mpp_init MJPEG decoder failed\n");
    mpp_destroy(ctx);
    return false;
  }
  mpp_ctx_ = ctx;
  mpp_mpi_ = api;

  // ── 2. Pre-allocate decoder output frame + buffer (required for MJPEG advanced API) ──
  RK_U32 hor_stride = (max_width  + 15) & ~15;
  RK_U32 ver_stride = (max_height + 15) & ~15;
  // JPEG can output YUV420 or YUV422; allocate 4x to cover all cases
  size_t frm_buf_size = hor_stride * ver_stride * 4;

  MppBufferGroup frm_grp = nullptr;
  if (mpp_buffer_group_get_internal(&frm_grp, MPP_BUFFER_TYPE_ION) != MPP_OK || !frm_grp) {
    MLOG_ERR("[mpp_cam] ERROR: ION buffer group alloc failed (frame) — check /dev/ion permissions\n");
    destroy();
    return false;
  }

  MppBuffer frm_buf = nullptr;
  if (mpp_buffer_get(frm_grp, &frm_buf, frm_buf_size) != MPP_OK || !frm_buf) {
    MLOG_ERR("[mpp_cam] ERROR: frame buffer alloc failed (%zu bytes) — ION memory exhausted?\n", frm_buf_size);
    mpp_buffer_group_put(frm_grp);
    destroy();
    return false;
  }

  MppFrame frame = nullptr;
  if (mpp_frame_init(&frame) != MPP_OK || !frame) {
    MLOG_ERR("[mpp_cam] ERROR: mpp_frame_init failed\n");
    mpp_buffer_put(frm_buf);
    mpp_buffer_group_put(frm_grp);
    destroy();
    return false;
  }
  mpp_frame_set_buffer(frame, frm_buf);

  dec_frm_grp_ = frm_grp;
  dec_frm_buf_ = frm_buf;
  dec_frame_   = frame;

  MLOG("[mpp] decoder output buffer: %zu bytes (stride %ux%u)\n",
       frm_buf_size, hor_stride, ver_stride);

  // ── 2b. Pre-allocate input packet buffer (avoids per-frame ION alloc/free) ──
  // MJPEG worst case is roughly max_width * max_height bytes (uncompressed JPEG
  // is always smaller); cap at a reasonable max to avoid wasting ION memory.
  pkt_buf_capacity_ = static_cast<size_t>(max_width) * max_height;
  {
    MppBufferGroup pkt_grp = nullptr;
    if (mpp_buffer_group_get_internal(&pkt_grp, MPP_BUFFER_TYPE_ION) != MPP_OK || !pkt_grp) {
      MLOG_ERR("[mpp_cam] ERROR: ION buffer group alloc failed (packet input)\n");
      destroy();
      return false;
    }
    MppBuffer pkt_buf = nullptr;
    if (mpp_buffer_get(pkt_grp, &pkt_buf, pkt_buf_capacity_) != MPP_OK || !pkt_buf) {
      MLOG_ERR("[mpp_cam] ERROR: packet input buffer alloc failed (%zu bytes)\n", pkt_buf_capacity_);
      mpp_buffer_group_put(pkt_grp);
      destroy();
      return false;
    }
    pkt_buf_grp_ = pkt_grp;
    pkt_mpp_buf_ = pkt_buf;
    MLOG_DEBUG("[mpp_cam] Pre-allocated input packet buffer: %zu bytes (ION)\n",
               pkt_buf_capacity_);
  }

  // ── 3. Pre-allocate DMA-backed RGB output buffer for RGA (at effective output resolution) ──
  int dst_w_stride = (eff_w + 15) & ~15;
  int dst_h_stride = (eff_h + 15) & ~15;
  dst_row_stride_pixels_ = dst_w_stride;
  dst_size_ = static_cast<size_t>(dst_w_stride) * dst_h_stride * 3;

  MppBufferGroup rgb_grp = nullptr;
  if (mpp_buffer_group_get_internal(&rgb_grp, MPP_BUFFER_TYPE_ION) != MPP_OK || !rgb_grp) {
    MLOG_ERR("[mpp_cam] ERROR: ION buffer group alloc failed (RGB output)\n");
    destroy();
    return false;
  }
  MppBuffer rgb_buf = nullptr;
  if (mpp_buffer_get(rgb_grp, &rgb_buf, dst_size_) != MPP_OK || !rgb_buf) {
    MLOG_ERR("[mpp_cam] ERROR: RGB buffer alloc failed (%zu bytes) — ION memory exhausted?\n", dst_size_);
    mpp_buffer_group_put(rgb_grp);
    destroy();
    return false;
  }

  dst_buf_grp_ = rgb_grp;
  dst_mpp_buf_ = rgb_buf;
  dst_fd_      = mpp_buffer_get_fd(rgb_buf);
  dst_ptr_     = mpp_buffer_get_ptr(rgb_buf);

  if (dst_fd_ < 0 || !dst_ptr_) {
    MLOG_ERR("[mpp_cam] ERROR: RGB DMA buffer fd=%d ptr=%p invalid\n", dst_fd_, dst_ptr_);
    destroy();
    return false;
  }

  // ── 4. Pre-import buffers into RGA ──────────────────────────────────
#if HAVE_RGA_IM2D
  rga_buffer_handle_t h = importbuffer_fd(dst_fd_, static_cast<int>(dst_size_));
  if (h > 0) {
    dst_rga_handle_ = static_cast<int>(h);
  }

  // Pre-import decode output buffer (source side) so we don't call
  // importbuffer_fd / releasebuffer_handle every frame.
  // Per-frame import/release leaks kernel RGA handles and eventually
  // deadlocks the RGA hardware — the root cause of the "camera stops
  // after frame #1 or #2" hang that persists across process restarts.
  {
    int dec_fd = mpp_buffer_get_fd(static_cast<MppBuffer>(dec_frm_buf_));
    if (dec_fd >= 0) {
      int src_buf_size = static_cast<int>(frm_buf_size);
      rga_buffer_handle_t sh = importbuffer_fd(dec_fd, src_buf_size);
      if (sh > 0) {
        src_rga_handle_ = static_cast<int>(sh);
        src_rga_fd_ = dec_fd;
        MLOG_DEBUG("[mpp_cam] Pre-imported decode output RGA handle=%d "
                   "(fd=%d, %d bytes)\n",
                   src_rga_handle_, dec_fd, src_buf_size);
      } else {
        MLOG_WARN("[mpp_cam] WARNING: failed to pre-import decode output buffer "
                  "into RGA (fd=%d) — will import per-frame (may leak handles)\n",
                  dec_fd);
      }
    }
  }
#endif

  initialized_ = true;
  if (resize_enabled_) {
    MLOG_INFO("[mpp_cam] MPP/RGA pipeline init OK: MJPEG advanced API, "
              "%dx%d -> RGA resize %dx%d\n",
              max_width, max_height, eff_w, eff_h);
  } else {
    MLOG_INFO("[mpp_cam] MPP/RGA pipeline init OK: MJPEG advanced API, "
              "%dx%d\n",
              max_width, max_height);
  }
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
void MppRgaPipeline::destroy()
{
  if (!initialized_ && !mpp_ctx_) return;

#if HAVE_RGA_IM2D
  if (ext_dst_rga_handle_ > 0) {
    releasebuffer_handle(static_cast<rga_buffer_handle_t>(ext_dst_rga_handle_));
    ext_dst_rga_handle_ = -1;
  }
  ext_dst_fd_ = -1;
  ext_dst_size_ = 0;
  ext_dst_vaddr_ = nullptr;
  ext_dst_byte_offset_ = 0;
  ext_dst_content_w_ = 0;
  ext_dst_content_h_ = 0;
  ext_dst_stride_w_ = 0;
  ext_dst_total_h_ = 0;
  ext_dst_x_offset_ = 0;
  ext_dst_y_offset_ = 0;
  if (src_rga_handle_ > 0) {
    releasebuffer_handle(static_cast<rga_buffer_handle_t>(src_rga_handle_));
    src_rga_handle_ = -1;
  }
  src_rga_fd_ = -1;
  if (dst_rga_handle_ > 0) {
    releasebuffer_handle(static_cast<rga_buffer_handle_t>(dst_rga_handle_));
    dst_rga_handle_ = -1;
  }
#endif
  resize_enabled_ = false;

  if (dst_mpp_buf_) {
    mpp_buffer_put(static_cast<MppBuffer>(dst_mpp_buf_));
    dst_mpp_buf_ = nullptr;
  }
  if (dst_buf_grp_) {
    mpp_buffer_group_put(static_cast<MppBufferGroup>(dst_buf_grp_));
    dst_buf_grp_ = nullptr;
  }
  dst_fd_ = -1; dst_ptr_ = nullptr; dst_size_ = 0;

  if (pkt_mpp_buf_) {
    mpp_buffer_put(static_cast<MppBuffer>(pkt_mpp_buf_));
    pkt_mpp_buf_ = nullptr;
  }
  if (pkt_buf_grp_) {
    mpp_buffer_group_put(static_cast<MppBufferGroup>(pkt_buf_grp_));
    pkt_buf_grp_ = nullptr;
  }
  pkt_buf_capacity_ = 0;

  if (dec_frame_) {
    mpp_frame_deinit(reinterpret_cast<MppFrame*>(&dec_frame_));
    dec_frame_ = nullptr;
  }
  if (dec_frm_buf_) {
    mpp_buffer_put(static_cast<MppBuffer>(dec_frm_buf_));
    dec_frm_buf_ = nullptr;
  }
  if (dec_frm_grp_) {
    mpp_buffer_group_put(static_cast<MppBufferGroup>(dec_frm_grp_));
    dec_frm_grp_ = nullptr;
  }

  if (mpp_ctx_) {
    mpp_destroy(static_cast<MppCtx>(mpp_ctx_));
    mpp_ctx_ = nullptr;
  }
  mpp_mpi_ = nullptr;
  initialized_ = false;
}

// ─────────────────────────────────────────────────────────────────────────────
bool MppRgaPipeline::setExternalDstDma(int fd, uint32_t total_size,
                                        void* vaddr, uint32_t byte_offset,
                                        int content_w, int content_h,
                                        int stride_w, int total_h)
{
#if HAVE_RGA_IM2D
  if (ext_dst_rga_handle_ > 0) {
    releasebuffer_handle(static_cast<rga_buffer_handle_t>(ext_dst_rga_handle_));
    ext_dst_rga_handle_ = -1;
  }
  ext_dst_fd_ = -1;
  ext_dst_size_ = 0;
  ext_dst_vaddr_ = nullptr;
  ext_dst_byte_offset_ = 0;
  ext_dst_content_w_ = 0;
  ext_dst_content_h_ = 0;
  ext_dst_stride_w_ = 0;
  ext_dst_total_h_ = 0;
  ext_dst_x_offset_ = 0;
  ext_dst_y_offset_ = 0;

  if (fd < 0) return true;

  // Use importbuffer_fd (not importbuffer_virtualaddr) for reliable DMA
  // mapping. importbuffer_virtualaddr fails intermittently on RKNN ION
  // buffers because the RGA kernel driver's dma_sync_single_for_device
  // can access invalid physical pages from get_user_pages on ION/DMA-BUF
  // backed memory, causing a kernel Oops.
  rga_buffer_handle_t h = importbuffer_fd(fd, static_cast<int>(total_size));
  if (h <= 0) {
    MLOG_ERR("[mpp_cam] setExternalDstDma: importbuffer_fd(fd=%d, %u) failed\n",
             fd, total_size);
    return false;
  }
  ext_dst_rga_handle_ = static_cast<int>(h);
  ext_dst_fd_ = fd;
  ext_dst_size_ = total_size;
  ext_dst_vaddr_ = vaddr;
  ext_dst_byte_offset_ = byte_offset;
  ext_dst_content_w_ = content_w;
  ext_dst_content_h_ = content_h;
  ext_dst_stride_w_ = stride_w;
  ext_dst_total_h_ = total_h;
  if (stride_w > 0 && byte_offset > 0) {
    const uint32_t row_bytes = static_cast<uint32_t>(stride_w) * 3;
    ext_dst_y_offset_ = static_cast<int>(byte_offset / row_bytes);
    ext_dst_x_offset_ = static_cast<int>((byte_offset % row_bytes) / 3);
  }
  MLOG_DEBUG("[mpp_cam] setExternalDstDma: OK fd=%d total=%u handle=%d "
             "content=%dx%d stride_w=%d total_h=%d y_offset=%d\n",
             fd, total_size, ext_dst_rga_handle_, content_w, content_h,
             stride_w, total_h, ext_dst_y_offset_);
  return true;
#else
  (void)fd; (void)total_size; (void)vaddr; (void)byte_offset;
  (void)content_w; (void)content_h; (void)stride_w; (void)total_h;
  return false;
#endif
}

// ─────────────────────────────────────────────────────────────────────────────
void MppRgaPipeline::switchExternalDstHandle(int rga_handle, void* vaddr)
{
  ext_dst_rga_handle_ = rga_handle;
  if (vaddr) ext_dst_vaddr_ = vaddr;
}

// ─────────────────────────────────────────────────────────────────────────────
int MppRgaPipeline::importExternalBuffer(int fd, uint32_t total_size)
{
#if HAVE_RGA_IM2D
  if (fd < 0 || total_size == 0) return -1;
  rga_buffer_handle_t h = importbuffer_fd(fd, static_cast<int>(total_size));
  return (h > 0) ? static_cast<int>(h) : -1;
#else
  (void)fd; (void)total_size;
  return -1;
#endif
}

void MppRgaPipeline::releaseExternalBuffer(int rga_handle)
{
#if HAVE_RGA_IM2D
  if (rga_handle > 0) {
    releasebuffer_handle(static_cast<rga_buffer_handle_t>(rga_handle));
  }
#else
  (void)rga_handle;
#endif
}

// ─────────────────────────────────────────────────────────────────────────────
bool MppRgaPipeline::decodeAndConvert(
    const uint8_t* mjpeg_data,
    size_t         mjpeg_size,
    uint8_t*       rgb_out,
    int*           out_width,
    int*           out_height)
{
  if (!initialized_ || !mjpeg_data || mjpeg_size == 0 || !out_width || !out_height)
    return false;

  // ── Software decode fallback (OpenCV imdecode) ────────────────────────
  if (use_sw_decode_) {
    if (!rgb_out) return false;
    cv::Mat jpeg_buf(1, static_cast<int>(mjpeg_size), CV_8UC1,
                     const_cast<uint8_t*>(mjpeg_data));
    cv::Mat bgr = cv::imdecode(jpeg_buf, cv::IMREAD_COLOR);
    if (bgr.empty()) return false;
    if (bgr.cols > max_width_ || bgr.rows > max_height_) return false;

    if (resize_enabled_) {
      cv::Mat bgr_resized;
      cv::resize(bgr, bgr_resized, cv::Size(out_width_, out_height_), 0, 0, cv::INTER_LINEAR);
      cv::Mat rgb_mat(out_height_, out_width_, CV_8UC3, rgb_out);
      cv::cvtColor(bgr_resized, rgb_mat, cv::COLOR_BGR2RGB);
      *out_width  = out_width_;
      *out_height = out_height_;
    } else {
      cv::Mat rgb_mat(bgr.rows, bgr.cols, CV_8UC3, rgb_out);
      cv::cvtColor(bgr, rgb_mat, cv::COLOR_BGR2RGB);
      *out_width  = bgr.cols;
      *out_height = bgr.rows;
    }
    return true;
  }

  // ── Hardware decode: MJPEG advanced (task-based) API ───────────────────
  MppCtx  ctx = static_cast<MppCtx>(mpp_ctx_);
  MppApi* mpi = static_cast<MppApi*>(mpp_mpi_);
  MppFrame out_frame = static_cast<MppFrame>(dec_frame_);
  MPP_RET ret;

  // 1. Copy MJPEG bytes into pre-allocated input MppBuffer (avoids per-frame ION alloc/free)
  MppBuffer pkt_buf = static_cast<MppBuffer>(pkt_mpp_buf_);
  MppPacket packet = nullptr;

  if (!pkt_buf || mjpeg_size > pkt_buf_capacity_) {
    MLOG_ERR("[mpp_cam] ERROR: MJPEG frame too large (%zu > capacity %zu)\n",
             mjpeg_size, pkt_buf_capacity_);
    goto hw_failed;
  }

  memcpy(mpp_buffer_get_ptr(pkt_buf), mjpeg_data, mjpeg_size);

  ret = mpp_packet_init_with_buffer(&packet, pkt_buf);
  if (ret != MPP_OK || !packet) {
    goto hw_failed;
  }
  mpp_packet_set_length(packet, mjpeg_size);

  MLOG("[mpp] advanced: packet %zu bytes\n", mjpeg_size);

  {
    static uint64_t hw_frame_idx = 0;
    ++hw_frame_idx;
    const bool verbose = (hw_frame_idx <= 5) || (hw_frame_idx % 3000 == 0);
    if (verbose) {
      MLOG_DEBUG("[mpp_cam] decodeAndConvert: frame #%lu, %zu bytes — "
                 "entering poll INPUT...\n",
                 (unsigned long)hw_frame_idx, mjpeg_size);
    }

    // 2. Poll input port with timeout (500ms) to avoid blocking the main thread
    //    indefinitely if a prior task was not reclaimed properly.
    constexpr MppPollType kPollTimeoutMs = static_cast<MppPollType>(500);
    ret = mpi->poll(ctx, MPP_PORT_INPUT, kPollTimeoutMs);
    if (ret != MPP_OK) {
      MLOG_ERR("[mpp_cam] ERROR: poll INPUT timeout/failed (ret=%d, frame #%lu) "
               "— MPP decoder may be stuck or resource leaked\n",
               ret, (unsigned long)hw_frame_idx);
      mpp_packet_deinit(&packet);
      goto hw_failed;
    }

    if (verbose) {
      MLOG_DEBUG("[mpp_cam] decodeAndConvert: frame #%lu — poll INPUT OK\n",
                 (unsigned long)hw_frame_idx);
    }

    // 3. Dequeue input task
    MppTask task_in = nullptr;
    ret = mpi->dequeue(ctx, MPP_PORT_INPUT, &task_in);
    if (ret != MPP_OK || !task_in) {
      MLOG_ERR("[mpp_cam] dequeue INPUT failed %d (frame #%lu)\n", ret, (unsigned long)hw_frame_idx);
      mpp_packet_deinit(&packet);
      goto hw_failed;
    }

    // 4. Set input packet and output frame
    mpp_task_meta_set_packet(task_in, KEY_INPUT_PACKET, packet);
    mpp_task_meta_set_frame(task_in, KEY_OUTPUT_FRAME, out_frame);

    // 5. Enqueue input task (triggers HW decode)
    ret = mpi->enqueue(ctx, MPP_PORT_INPUT, task_in);
    if (ret != MPP_OK) {
      MLOG_ERR("[mpp_cam] enqueue INPUT failed %d (frame #%lu)\n", ret, (unsigned long)hw_frame_idx);
      mpp_packet_deinit(&packet);
      goto hw_failed;
    }

    if (verbose) {
      MLOG_DEBUG("[mpp_cam] decodeAndConvert: frame #%lu — entering poll "
                 "OUTPUT...\n",
                 (unsigned long)hw_frame_idx);
    }

    // 6. Poll output port with timeout (500ms)
    ret = mpi->poll(ctx, MPP_PORT_OUTPUT, kPollTimeoutMs);
    if (ret != MPP_OK) {
      MLOG_ERR("[mpp_cam] ERROR: poll OUTPUT timeout/failed (ret=%d, frame #%lu) "
               "— MPP decoder may be stuck\n",
               ret, (unsigned long)hw_frame_idx);
      mpp_packet_deinit(&packet);
      goto hw_failed;
    }

    if (verbose) {
      MLOG_DEBUG("[mpp_cam] decodeAndConvert: frame #%lu — poll OUTPUT OK\n",
                 (unsigned long)hw_frame_idx);
    }

    // 7. Dequeue output task
    MppTask task_out = nullptr;
    ret = mpi->dequeue(ctx, MPP_PORT_OUTPUT, &task_out);
    if (ret != MPP_OK || !task_out) {
      MLOG_ERR("[mpp_cam] dequeue OUTPUT failed %d\n", ret);
      mpp_packet_deinit(&packet);
      goto hw_failed;
    }

    // 8. Get decoded frame
    MppFrame frame_out = nullptr;
    mpp_task_meta_get_frame(task_out, KEY_OUTPUT_FRAME, &frame_out);

    // Reclaim input task and release packet (packet only — buffer is pre-allocated).
    // Without reclaim + re-enqueue, the next poll(INPUT) will block/timeout because
    // the MPP task pool is exhausted.
    {
      MppTask task_reclaim = nullptr;
      ret = mpi->dequeue(ctx, MPP_PORT_INPUT, &task_reclaim);
      if (ret != MPP_OK || !task_reclaim) {
        MLOG_WARN("[mpp_cam] WARNING: input task reclaim failed (ret=%d, "
                  "frame #%lu) — next poll(INPUT) will likely timeout\n",
                  ret, (unsigned long)hw_frame_idx);
      } else {
        MppPacket pkt_out = nullptr;
        mpp_task_meta_get_packet(task_reclaim, KEY_INPUT_PACKET, &pkt_out);
        if (pkt_out) mpp_packet_deinit(&pkt_out);
        mpi->enqueue(ctx, MPP_PORT_INPUT, task_reclaim);
      }
    }

    if (!frame_out) {
      MLOG("[mpp] advanced: no output frame\n");
      mpi->enqueue(ctx, MPP_PORT_OUTPUT, task_out);
      goto hw_failed;
    }

    MLOG("[mpp] advanced: decode OK\n");

    // 9. Extract decoded frame geometry and pixel format
    MppBuffer src_buf  = mpp_frame_get_buffer(frame_out);
    int actual_w   = mpp_frame_get_width(frame_out);
    int actual_h   = mpp_frame_get_height(frame_out);
    int hor_stride = mpp_frame_get_hor_stride(frame_out);
    int ver_stride = mpp_frame_get_ver_stride(frame_out);
    MppFrameFormat frame_fmt = mpp_frame_get_fmt(frame_out);

    // MPP format enum: 0=YUV420SP(NV12), 1=YUV420SP_10BIT, 2=YUV422SP(NV16),
    // 4=YUV420P(I420), 5=YUV420SP_VU(NV21), 7=YUV422SP_VU(NV61)
    if (hw_frame_idx <= 3 || (hw_frame_idx % 3000) == 0) {
      MLOG_DEBUG("[mpp_cam] frame #%lu: %dx%d stride %dx%d fmt=%d "
                 "(0=NV12 2=NV16/YUV422SP 5=NV21)\n",
                 (unsigned long)hw_frame_idx, actual_w, actual_h,
                 hor_stride, ver_stride, static_cast<int>(frame_fmt));
    }

    MLOG("[mpp] frame: %dx%d stride %dx%d fmt=%d\n",
         actual_w, actual_h, hor_stride, ver_stride,
         static_cast<int>(frame_fmt));

    const bool is_yuv422sp = (frame_fmt == MPP_FMT_YUV422SP ||
                              frame_fmt == MPP_FMT_YUV422SP_VU);

    if (!src_buf || actual_w <= 0 || actual_h <= 0 ||
        hor_stride <= 0 || ver_stride <= 0 ||
        actual_w > max_width_ || actual_h > max_height_) {
      mpi->enqueue(ctx, MPP_PORT_OUTPUT, task_out);
      goto hw_failed;
    }

    // 10. Color conversion: YUV → RGB (with optional RGA resize)
    // Detect source format: NV16 (YUV422SP) vs NV12 (YUV420SP)
    const int rga_src_fmt = is_yuv422sp
        ? RK_FORMAT_YCbCr_422_SP : RK_FORMAT_YCbCr_420_SP;
    // NV16: chroma plane is full height; NV12: chroma plane is half height
    const int src_yuv_size = is_yuv422sp
        ? hor_stride * ver_stride * 2
        : hor_stride * ver_stride * 3 / 2;

    bool converted = false;
    int final_w = actual_w;
    int final_h = actual_h;

#if HAVE_RGA_IM2D
    {
      // Use the pre-imported source RGA handle (created once in init()).
      // Falling back to per-frame import only if pre-import failed at init.
      rga_buffer_handle_t src_handle = 0;
      bool src_handle_is_temp = false;
      const int src_fd = mpp_buffer_get_fd(src_buf);
      if (src_rga_handle_ > 0 && src_fd == src_rga_fd_) {
        src_handle = static_cast<rga_buffer_handle_t>(src_rga_handle_);
      } else if (src_fd >= 0) {
        if (src_rga_handle_ > 0 && src_fd != src_rga_fd_ &&
            (hw_frame_idx <= 5 || (hw_frame_idx % 3000) == 0)) {
          MLOG_DEBUG("[mpp_cam] frame #%lu: MPP output fd changed %d -> %d; "
                     "using temporary RGA import\n",
                     (unsigned long)hw_frame_idx, src_rga_fd_, src_fd);
        }
        src_handle = importbuffer_fd(src_fd, src_yuv_size);
        src_handle_is_temp = (src_handle > 0);
      }

      if (src_handle > 0 && dst_rga_handle_ > 0) {
        int dst_w = resize_enabled_ ? out_width_  : actual_w;
        int dst_h = resize_enabled_ ? out_height_ : actual_h;

        rga_buffer_t src_img = wrapbuffer_handle(
            src_handle, actual_w, actual_h, rga_src_fmt);
        src_img.wstride = hor_stride;
        src_img.hstride = ver_stride;

        IM_STATUS rga_ret = IM_STATUS_FAILED;
        const bool use_ext_dma = (ext_dst_rga_handle_ > 0);
        bool use_ext_offset_fallback = false;
        if (use_ext_dma) {
          use_ext_offset_fallback =
              (ext_dst_x_offset_ != 0 || ext_dst_y_offset_ != 0);
          const int cvt_dst_handle = use_ext_offset_fallback
              ? dst_rga_handle_ : ext_dst_rga_handle_;
          rga_buffer_t dst_img = wrapbuffer_handle(
              static_cast<rga_buffer_handle_t>(cvt_dst_handle),
              dst_w, dst_h, RK_FORMAT_RGB_888);
          if (hw_frame_idx <= 2) {
            MLOG_DEBUG("[mpp_cam] zero-copy ext_offset_fallback=%d\n",
                       use_ext_offset_fallback ? 1 : 0);
          }
          if (use_ext_offset_fallback) {
            // Match the pre cross-process zero-copy pipeline: RGA writes the
            // resized RGB content into the internal DMA buffer, then CPU copies
            // it into the RKNN input buffer at the letterbox offset. The old
            // integrated pipeline intentionally avoided RGA destination-rect
            // offsets for RKNN DMA buffers because that path is unreliable on
            // this BSP/librga combination.
            rga_ret = imcvtcolor(
                src_img, dst_img,
                rga_src_fmt, RK_FORMAT_RGB_888, IM_YUV_TO_RGB_BT601_FULL);
            if (rga_ret != IM_STATUS_SUCCESS) {
              MLOG_ERR("[mpp_cam] zero-copy offset fallback YUV->RGB FAILED ret=%d\n",
                       rga_ret);
            }
          } else {
            rga_ret = imcvtcolor(
                src_img, dst_img,
                rga_src_fmt, RK_FORMAT_RGB_888, IM_YUV_TO_RGB_BT601_FULL);
          }
          if (rga_ret == IM_STATUS_SUCCESS) {
            final_w = dst_w;
            final_h = dst_h;
          }
        } else {
          rga_buffer_t dst_img = wrapbuffer_handle(
              static_cast<rga_buffer_handle_t>(dst_rga_handle_),
              dst_w, dst_h, RK_FORMAT_RGB_888);

          // V4L2 reports MJPEG stream quantization as full range.
          // Using the default CSC here maps luma as limited range and causes
          // black crush (many pixels clip to 0) on /top/image_raw.
          rga_ret = imcvtcolor(
              src_img, dst_img,
              rga_src_fmt, RK_FORMAT_RGB_888, IM_YUV_TO_RGB_BT601_FULL);
        }

        if (src_handle_is_temp) {
          releasebuffer_handle(src_handle);
        }

        if (rga_ret == IM_STATUS_SUCCESS) {
          if (use_ext_dma && use_ext_offset_fallback && ext_dst_vaddr_) {
            uint8_t* dst_at_offset = static_cast<uint8_t*>(ext_dst_vaddr_) +
                                     ext_dst_byte_offset_;
            copyPackedRgb(dst_at_offset, static_cast<const uint8_t*>(dst_ptr_),
                          dst_w, dst_h, dst_row_stride_pixels_);
          } else if (use_ext_dma && use_ext_offset_fallback) {
            MLOG_ERR("[mpp_cam] zero-copy offset fallback missing ext dst vaddr\n");
            converted = false;
            rga_ret = IM_STATUS_FAILED;
          } else if (rgb_out && ext_dst_rga_handle_ <= 0) {
            copyPackedRgb(rgb_out, static_cast<const uint8_t*>(dst_ptr_), dst_w,
                          dst_h, dst_row_stride_pixels_);
          }
          converted = (rga_ret == IM_STATUS_SUCCESS);
          if (!use_ext_dma) {
            final_w = dst_w;
            final_h = dst_h;
          }
          if (converted && hw_frame_idx <= 5) {
            const char* tag = "";
            if (use_ext_dma) {
              tag = use_ext_offset_fallback
                  ? " [ext DMA+memcpy offset fallback]" : " [ext DMA direct]";
            }
            MLOG_DEBUG("[mpp_cam] frame #%lu: RGA imcvtcolor OK "
                       "(%dx%d fmt=%d -> %dx%d RGB)%s\n",
                       (unsigned long)hw_frame_idx, actual_w, actual_h,
                       rga_src_fmt, dst_w, dst_h, tag);
          }
        } else {
          MLOG_ERR("[mpp_cam] RGA imcvtcolor FAILED (%d, %dx%d fmt=%d -> %dx%d RGB)\n",
                   rga_ret, actual_w, actual_h, rga_src_fmt, dst_w, dst_h);
        }
      } else {
        if (src_handle_is_temp) {
          releasebuffer_handle(src_handle);
        }
      }
    }
#endif

    if (!converted && rgb_out) {
      // CPU fallback: YUV → RGB (with optional OpenCV resize)
      void* yuv_ptr = mpp_buffer_get_ptr(src_buf);
      if (yuv_ptr) {
        if (is_yuv422sp) {
          // NV16 (YUV422 Semi-Planar): Y plane [h*stride] + interleaved UV [h*stride]
          // OpenCV has no direct NV16 support, so we deinterleave UV manually
          // and use cvtColor with YUV2RGB (3-plane YUV422P).
          uint8_t* y_plane = static_cast<uint8_t*>(yuv_ptr);
          uint8_t* uv_plane = y_plane + hor_stride * ver_stride;

          cv::Mat y_mat(actual_h, actual_w, CV_8UC1);
          cv::Mat u_mat(actual_h, actual_w / 2, CV_8UC1);
          cv::Mat v_mat(actual_h, actual_w / 2, CV_8UC1);

          for (int row = 0; row < actual_h; ++row) {
            memcpy(y_mat.ptr(row), y_plane + row * hor_stride,
                   static_cast<size_t>(actual_w));
            const uint8_t* uv_row = uv_plane + row * hor_stride;
            uint8_t* u_row = u_mat.ptr(row);
            uint8_t* v_row = v_mat.ptr(row);
            for (int x = 0; x < actual_w / 2; ++x) {
              u_row[x] = uv_row[2 * x];
              v_row[x] = uv_row[2 * x + 1];
            }
          }

          // Upsample U/V horizontally and merge to YUV 3-channel
          cv::Mat u_full, v_full;
          cv::resize(u_mat, u_full, cv::Size(actual_w, actual_h), 0, 0, cv::INTER_LINEAR);
          cv::resize(v_mat, v_full, cv::Size(actual_w, actual_h), 0, 0, cv::INTER_LINEAR);

          cv::Mat yuv_merged;
          std::vector<cv::Mat> channels = {y_mat, u_full, v_full};
          cv::merge(channels, yuv_merged);
          cv::Mat rgb_full;
          cv::cvtColor(yuv_merged, rgb_full, cv::COLOR_YUV2RGB);

          if (resize_enabled_) {
            cv::Mat rgb_resized;
            cv::resize(rgb_full, rgb_resized, cv::Size(out_width_, out_height_), 0, 0, cv::INTER_LINEAR);
            memcpy(rgb_out, rgb_resized.data, static_cast<size_t>(out_width_) * out_height_ * 3);
            final_w = out_width_;
            final_h = out_height_;
          } else {
            memcpy(rgb_out, rgb_full.data, static_cast<size_t>(actual_w) * actual_h * 3);
          }
          converted = true;
        } else {
          // NV12 path (original)
          cv::Mat nv12_mat(ver_stride + ver_stride / 2, hor_stride,
                           CV_8UC1, yuv_ptr);
          if (resize_enabled_) {
            cv::Mat rgb_full;
            if (hor_stride == actual_w) {
              cv::cvtColor(nv12_mat, rgb_full, cv::COLOR_YUV2RGB_NV12);
            } else {
              cv::Mat rgb_wide(actual_h, hor_stride, CV_8UC3);
              cv::cvtColor(nv12_mat, rgb_wide, cv::COLOR_YUV2RGB_NV12);
              rgb_full = rgb_wide(cv::Rect(0, 0, actual_w, actual_h)).clone();
            }
            cv::Mat rgb_resized;
            cv::resize(rgb_full, rgb_resized, cv::Size(out_width_, out_height_), 0, 0, cv::INTER_LINEAR);
            memcpy(rgb_out, rgb_resized.data, static_cast<size_t>(out_width_) * out_height_ * 3);
            final_w = out_width_;
            final_h = out_height_;
          } else {
            cv::Mat rgb_mat(actual_h, actual_w, CV_8UC3, rgb_out);
            if (hor_stride == actual_w) {
              cv::cvtColor(nv12_mat, rgb_mat, cv::COLOR_YUV2RGB_NV12);
            } else {
              cv::Mat rgb_full(actual_h, hor_stride, CV_8UC3);
              cv::cvtColor(nv12_mat, rgb_full, cv::COLOR_YUV2RGB_NV12);
              rgb_full(cv::Rect(0, 0, actual_w, actual_h)).copyTo(rgb_mat);
            }
          }
          converted = true;
        }
        MLOG("[mpp] CPU YUV(fmt=%d)->RGB OK\n", static_cast<int>(frame_fmt));
      }
    }

    // Return the output task only after RGA/CPU has finished reading frame_out.
    mpi->enqueue(ctx, MPP_PORT_OUTPUT, task_out);

    if (!converted) goto hw_failed;

    mpp_hw_attempts_ = 0;
    *out_width  = final_w;
    *out_height = final_h;
    return true;
  }

hw_failed:
  ++mpp_hw_attempts_;
  if (mpp_hw_attempts_ >= kMaxMppHwAttempts) {
    MLOG_WARN("[mpp_cam] WARNING: HW decode failed %d times — switching to "
              "OpenCV software decode\n",
              mpp_hw_attempts_);
    use_sw_decode_ = true;
    return decodeAndConvert(mjpeg_data, mjpeg_size, rgb_out, out_width, out_height);
  }
  return false;
}

}  // namespace circle::perception

#endif  // USE_MPP_RGA
