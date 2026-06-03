#pragma once

#include <chrono>
#include <cstdint>
#include <vector>

#include "circle/perception/mpp_rga_pipeline.hpp"
#include "circle/perception/rknn_engine.hpp"

namespace circle::perception {

/** Per-slot DMA/RGA state shared by mpp_cam zero-copy and in-process bf_flight. */
struct ZeroCopyFrameSlot {
  int dma_fd{-1};
  int rga_handle{-1};
  void* mmap_ptr{nullptr};
  bool leased{false};
  std::chrono::steady_clock::time_point lease_time{};
  uint32_t seq{0};
};

struct ZeroCopySlotGeometry {
  uint32_t model_w{0};
  uint32_t model_h{0};
  uint32_t buffer_size{0};
  uint32_t byte_offset{0};
  int content_w{0};
  int content_h{0};
};

void syncDmaBufForCpuRead(int fd, bool start);
void syncDmaBufForCpuWrite(int fd, bool start);

/** Import RKNN input DMA fds as RGA destinations (one handle per slot). */
bool configureZeroCopySlots(MppRgaPipeline& pipeline,
                            const std::vector<int>& dma_fds,
                            uint32_t buffer_size,
                            int content_w,
                            int content_h,
                            int model_w,
                            int model_h,
                            uint32_t byte_offset,
                            std::vector<ZeroCopyFrameSlot>* slots,
                            std::string* error);

void releaseZeroCopySlots(std::vector<ZeroCopyFrameSlot>* slots,
                          uint32_t buffer_size);

int leaseFreeZeroCopySlot(std::vector<ZeroCopyFrameSlot>* slots);

void releaseZeroCopySlotLease(std::vector<ZeroCopyFrameSlot>* slots,
                              uint32_t slot_id);

}  // namespace circle::perception
