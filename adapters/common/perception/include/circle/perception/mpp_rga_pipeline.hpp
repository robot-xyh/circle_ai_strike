/**
 * @file mpp_rga_pipeline.hpp
 * @brief MJPEG -> MPP decode (NV12) -> RGA (RGB). Only built when USE_MPP_RGA.
 *
 * Hardware pipeline (MJPEG advanced/task-based API):
 *   MPP: MJPEG bytes -> copy to input DMA buffer -> task enqueue -> HW decode
 *        -> NV12 in pre-allocated output DMA buffer
 *   RGA: NV12 DMA fd -> imcvtcolor -> RGB DMA buffer
 *   CPU: memcpy RGB DMA buffer -> caller's rgb_out (skipped when rgb_out=nullptr)
 *
 * Optional zero-copy path: setExternalDstDma() redirects RGA's colour
 * conversion output to an external DMA buffer (e.g. RKNN NPU input). If the
 * target buffer needs a letterbox offset, the pipeline uses the legacy-safe
 * internal-RGB + CPU-copy fallback because RGA destination-rect offsets are
 * unreliable on this BSP/librga combination.
 *
 * MJPEG on Rockchip MPP requires the "advanced" (task-based) decode API:
 *   poll(INPUT) -> dequeue(INPUT) -> set packet+frame -> enqueue(INPUT)
 *   -> poll(OUTPUT) -> dequeue(OUTPUT) -> get frame -> enqueue(OUTPUT)
 * The "simple" API (decode_put_packet/decode_get_frame) returns
 * MPP_ERR_BUFFER_FULL (-1012) for MJPEG on BSP MPP 1.5.0.
 */

#ifndef CIRCLE_PERCEPTION__MPP_RGA_PIPELINE_HPP_
#define CIRCLE_PERCEPTION__MPP_RGA_PIPELINE_HPP_

#include <cstddef>
#include <cstdint>

namespace circle::perception {

enum class MppRgaLogLevel {
  Debug,
  Info,
  Warn,
  Error,
};

using MppRgaLogEnabledHandler = bool (*)(MppRgaLogLevel level);
using MppRgaLogHandler = void (*)(MppRgaLogLevel level, const char* message);

void setMppRgaLogHandler(MppRgaLogEnabledHandler enabled_handler,
                         MppRgaLogHandler log_handler);
bool mppRgaLogEnabled(MppRgaLogLevel level);
void mppRgaLog(MppRgaLogLevel level, const char* fmt, ...);

class MppRgaPipeline {
 public:
  MppRgaPipeline() = default;
  ~MppRgaPipeline();

  MppRgaPipeline(const MppRgaPipeline&)            = delete;
  MppRgaPipeline& operator=(const MppRgaPipeline&) = delete;
  MppRgaPipeline(MppRgaPipeline&&)                 = delete;
  MppRgaPipeline& operator=(MppRgaPipeline&&)      = delete;

  /**
   * @param max_width   V4L2 capture width  (sensor resolution)
   * @param max_height  V4L2 capture height (sensor resolution)
   * @param out_width   Published output width  (0 = same as capture, no resize)
   * @param out_height  Published output height (0 = same as capture, no resize)
   *
   * When out_width x out_height differs from capture, an RGA hardware resize
   * step is inserted between MPP decode and colour conversion, keeping full
   * sensor FOV while reducing downstream data volume.
   */
  bool init(int max_width, int max_height, int out_width = 0, int out_height = 0);
  void destroy();

  bool decodeAndConvert(
      const uint8_t* mjpeg_data,
      size_t mjpeg_size,
      uint8_t* rgb_out,
      int* out_width,
      int* out_height);

  bool isInitialized() const { return initialized_; }
  bool isResizeEnabled() const { return resize_enabled_; }
  int  outputWidth()  const { return resize_enabled_ ? out_width_  : max_width_;  }
  int  outputHeight() const { return resize_enabled_ ? out_height_ : max_height_; }

  /// Direct access to the RGA DMA output buffer (uncached ION memory).
  /// Valid after a successful decodeAndConvert() call.
  /// When rgb_out is nullptr, decodeAndConvert() skips the internal memcpy
  /// and the caller should read from getDstPtr() instead.
  uint8_t* getDstPtr()  const { return static_cast<uint8_t*>(dst_ptr_); }
  size_t   getDstSize() const { return dst_size_; }

  /**
   * Set an external DMA buffer as the RGA colour-conversion destination.
   * Uses importbuffer_fd (not importbuffer_virtualaddr) for reliable DMA
   * mapping through the kernel DMA-BUF framework.
   * @param fd           DMA fd backing the buffer
   * @param total_size   Total size of the DMA buffer in bytes
   * @param vaddr        Virtual address of the DMA buffer (for CPU memcpy
   *                     fallback when y-offset is needed; NOT used for RGA import)
   * @param byte_offset  Byte offset within the buffer where content starts
   *                     (e.g. top_pad * width * 3 for letterbox)
   * @param content_w    Width of the content region in pixels
   * @param content_h    Height of the content region in pixels
   * @param stride_w     Horizontal stride of the full buffer in pixels
   *                     (e.g. model_input_w)
   * @param total_h      Total height of the full buffer in pixels
   *                     (e.g. model_input_h)
   * @return true if RGA handle import succeeded
   */
  bool setExternalDstDma(int fd, uint32_t total_size,
                         void* vaddr, uint32_t byte_offset,
                         int content_w, int content_h,
                         int stride_w, int total_h);

  /**
   * Switch the active external DMA destination handle without re-importing.
   * Used by the pipeline overlap path to alternate between two pre-imported
   * RKNN input buffers (one per slot).
   * @param rga_handle  Pre-imported RGA handle (from importbuffer_fd).
   *                    Pass -1 to revert to the internal RGB DMA buffer.
   * @param vaddr       Base virtual address of the DMA buffer (for memcpy
   *                    when y-offset > 0). May be nullptr if offset == 0.
   */
  void switchExternalDstHandle(int rga_handle, void* vaddr = nullptr);

  /**
   * Import an additional external DMA buffer as an RGA handle via fd.
   * The caller owns the returned handle and must call releaseExternalBuffer().
   * @return Imported RGA handle (> 0), or -1 on failure.
   */
  static int importExternalBuffer(int fd, uint32_t total_size);
  static void releaseExternalBuffer(int rga_handle);

  /// Current active external RGA handle (-1 if using internal buffer).
  int externalDstHandle() const { return ext_dst_rga_handle_; }

 private:
  bool initialized_{false};
  int  max_width_{0};
  int  max_height_{0};

  // Output dimensions (after optional RGA resize)
  int  out_width_{0};
  int  out_height_{0};
  bool resize_enabled_{false};

  // MPP decoder context
  void* mpp_ctx_{nullptr};   // MppCtx
  void* mpp_mpi_{nullptr};   // MppApi*

  // MJPEG advanced API: pre-allocated output frame + buffer for decoder
  void* dec_frm_grp_{nullptr};   // MppBufferGroup (ION) for decoder output
  void* dec_frm_buf_{nullptr};   // MppBuffer for NV12 output
  void* dec_frame_{nullptr};     // MppFrame (reused each decode, holds dec_frm_buf_)

  // Pre-allocated input packet buffer (avoids per-frame ION alloc/free)
  void* pkt_buf_grp_{nullptr};   // MppBufferGroup (ION) for input MJPEG
  void* pkt_mpp_buf_{nullptr};   // MppBuffer for input MJPEG data
  size_t pkt_buf_capacity_{0};   // allocated capacity in bytes

  // Pre-allocated DMA-backed RGB output buffer for RGA (at output resolution)
  void*  dst_buf_grp_{nullptr};  // MppBufferGroup (ION)
  void*  dst_mpp_buf_{nullptr};  // MppBuffer
  int    dst_fd_{-1};
  void*  dst_ptr_{nullptr};
  size_t dst_size_{0};
  int    dst_row_stride_pixels_{0};

  // Pre-imported RGA handle for dst buffer
  int dst_rga_handle_{-1};

  // Pre-imported RGA handle for MPP decode output buffer (source side).
  // Imported once at init() and reused every frame to avoid per-frame
  // importbuffer_fd/releasebuffer_handle calls that leak kernel RGA
  // handles and eventually deadlock the RGA driver.
  int src_rga_handle_{-1};
  int src_rga_fd_{-1};

  // External DMA destination (e.g. RKNN input buffer) — used instead of dst_rga_handle_
  int ext_dst_rga_handle_{-1};
  int ext_dst_fd_{-1};
  uint32_t ext_dst_size_{0};
  void* ext_dst_vaddr_{nullptr};  // for CPU memcpy when y-offset > 0
  uint32_t ext_dst_byte_offset_{0};
  // Destination geometry for the external DMA buffer
  int ext_dst_content_w_{0};
  int ext_dst_content_h_{0};
  int ext_dst_stride_w_{0};
  int ext_dst_total_h_{0};
  int ext_dst_x_offset_{0};  // column offset for letterbox
  int ext_dst_y_offset_{0};  // row offset (top_pad) for letterbox
  // Software fallback
  bool use_sw_decode_{false};
  int  mpp_hw_attempts_{0};
  static constexpr int kMaxMppHwAttempts = 3;
};

}  // namespace circle::perception

#endif  // CIRCLE_PERCEPTION__MPP_RGA_PIPELINE_HPP_
