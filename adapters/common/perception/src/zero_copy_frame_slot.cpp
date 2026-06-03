#include "circle/perception/zero_copy_frame_slot.hpp"

#include <cerrno>
#include <cstring>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#ifdef __linux__
#include <linux/dma-buf.h>
#endif

namespace circle::perception {

void syncDmaBufForCpuRead(int fd, bool start) {
#ifdef DMA_BUF_IOCTL_SYNC
  if (fd < 0) {
    return;
  }
  dma_buf_sync sync{};
  sync.flags = DMA_BUF_SYNC_READ | (start ? DMA_BUF_SYNC_START : DMA_BUF_SYNC_END);
  (void)::ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync);
#else
  (void)fd;
  (void)start;
#endif
}

void syncDmaBufForCpuWrite(int fd, bool start) {
#ifdef DMA_BUF_IOCTL_SYNC
  if (fd < 0) {
    return;
  }
  dma_buf_sync sync{};
  sync.flags = DMA_BUF_SYNC_WRITE | (start ? DMA_BUF_SYNC_START : DMA_BUF_SYNC_END);
  (void)::ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync);
#else
  (void)fd;
  (void)start;
#endif
}

bool configureZeroCopySlots(MppRgaPipeline& pipeline,
                            const std::vector<int>& dma_fds,
                            uint32_t buffer_size,
                            int content_w,
                            int content_h,
                            int model_w,
                            int model_h,
                            uint32_t byte_offset,
                            std::vector<ZeroCopyFrameSlot>* slots,
                            std::string* error) {
  if (!slots || dma_fds.empty() || buffer_size == 0 || content_w <= 0 ||
      content_h <= 0 || model_w <= 0 || model_h <= 0) {
    if (error) {
      *error = "invalid zero-copy slot geometry";
    }
    return false;
  }

  releaseZeroCopySlots(slots, buffer_size);
  slots->clear();
  slots->reserve(dma_fds.size());

#if CIRCLE_PERCEPTION_USE_MPP_RGA
  if (!pipeline.setExternalDstDma(dma_fds[0], buffer_size, nullptr, byte_offset,
                                  content_w, content_h, model_w, model_h)) {
    if (error) {
      *error = "setExternalDstDma failed for slot 0";
    }
    return false;
  }

  ZeroCopyFrameSlot first;
  first.dma_fd = dma_fds[0];
  first.rga_handle = pipeline.externalDstHandle();
  first.mmap_ptr =
      ::mmap(nullptr, buffer_size, PROT_READ | PROT_WRITE, MAP_SHARED, dma_fds[0], 0);
  if (first.mmap_ptr == MAP_FAILED) {
    first.mmap_ptr = nullptr;
    if (byte_offset > 0) {
      pipeline.switchExternalDstHandle(-1);
      if (first.rga_handle > 0) {
        MppRgaPipeline::releaseExternalBuffer(first.rga_handle);
      }
      if (error) {
        *error = "mmap failed for slot 0";
      }
      return false;
    }
  }
  slots->push_back(first);

  for (size_t i = 1; i < dma_fds.size(); ++i) {
    const int handle = MppRgaPipeline::importExternalBuffer(dma_fds[i], buffer_size);
    if (handle <= 0) {
      pipeline.switchExternalDstHandle(-1);
      releaseZeroCopySlots(slots, buffer_size);
      if (error) {
        *error = "importExternalBuffer failed for slot " + std::to_string(i);
      }
      return false;
    }
    ZeroCopyFrameSlot slot;
    slot.dma_fd = dma_fds[i];
    slot.rga_handle = handle;
    slot.mmap_ptr =
        ::mmap(nullptr, buffer_size, PROT_READ | PROT_WRITE, MAP_SHARED, dma_fds[i], 0);
    if (slot.mmap_ptr == MAP_FAILED) {
      slot.mmap_ptr = nullptr;
      if (byte_offset > 0) {
        pipeline.switchExternalDstHandle(-1);
        releaseZeroCopySlots(slots, buffer_size);
        MppRgaPipeline::releaseExternalBuffer(handle);
        if (error) {
          *error = "mmap failed for slot " + std::to_string(i);
        }
        return false;
      }
    }
    slots->push_back(slot);
  }

  pipeline.switchExternalDstHandle((*slots)[0].rga_handle, (*slots)[0].mmap_ptr);
  return true;
#else
  (void)pipeline;
  (void)byte_offset;
  if (error) {
    *error = "MPP/RGA not available";
  }
  return false;
#endif
}

void releaseZeroCopySlots(std::vector<ZeroCopyFrameSlot>* slots,
                          uint32_t buffer_size) {
  if (!slots) {
    return;
  }
  for (auto& slot : *slots) {
    if (slot.mmap_ptr && buffer_size > 0) {
      ::munmap(slot.mmap_ptr, buffer_size);
      slot.mmap_ptr = nullptr;
    }
    if (slot.rga_handle > 0) {
      MppRgaPipeline::releaseExternalBuffer(slot.rga_handle);
      slot.rga_handle = -1;
    }
    slot.dma_fd = -1;
    slot.leased = false;
  }
}

int leaseFreeZeroCopySlot(std::vector<ZeroCopyFrameSlot>* slots) {
  if (!slots) {
    return -1;
  }
  for (size_t i = 0; i < slots->size(); ++i) {
    if (!(*slots)[i].leased) {
      (*slots)[i].leased = true;
      (*slots)[i].lease_time = std::chrono::steady_clock::now();
      return static_cast<int>(i);
    }
  }
  return -1;
}

void releaseZeroCopySlotLease(std::vector<ZeroCopyFrameSlot>* slots,
                              uint32_t slot_id) {
  if (!slots || slot_id >= slots->size()) {
    return;
  }
  (*slots)[slot_id].leased = false;
  (*slots)[slot_id].lease_time = {};
}

}  // namespace circle::perception
