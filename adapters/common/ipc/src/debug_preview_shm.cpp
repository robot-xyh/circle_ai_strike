#include "circle/ipc/debug_preview_shm.hpp"

#include <cerrno>
#include <cstring>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace circle::ipc {

namespace {
constexpr char kMagic[8] = {'C', 'P', 'D', 'B', 'G', 'P', 'V', '1'};
constexpr uint32_t kVersion = 2;
}  // namespace

DebugPreviewWriter::~DebugPreviewWriter() { close(); }

bool DebugPreviewWriter::open(const DebugPreviewWriterConfig& config) {
  close();
  config_ = config;
  if (config_.width == 0 || config_.height == 0 || config_.slot_count == 0) {
    return false;
  }
  const uint32_t stride = config_.width * 3u;
  const uint32_t pixel_bytes = stride * config_.height;
  const uint32_t slot_bytes = pixel_bytes +
      static_cast<uint32_t>(sizeof(circle::debug::PreviewOverlayShmData));
  const size_t total = sizeof(DebugPreviewShmHeader) +
                       static_cast<size_t>(config_.slot_count) * slot_bytes;

  fd_ = ::shm_open(config_.shm_name.c_str(), O_CREAT | O_RDWR, 0600);
  if (fd_ < 0) {
    return false;
  }
  if (::ftruncate(fd_, static_cast<off_t>(total)) != 0) {
    close();
    return false;
  }
  mapping_size_ = total;
  mapping_ = ::mmap(nullptr, total, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
  if (mapping_ == MAP_FAILED) {
    mapping_ = nullptr;
    close();
    return false;
  }
  header_ = reinterpret_cast<DebugPreviewShmHeader*>(mapping_);
  std::memset(header_, 0, sizeof(*header_));
  std::memcpy(header_->magic, kMagic, sizeof(kMagic));
  header_->version = kVersion;
  header_->slot_count = config_.slot_count;
  header_->slot_bytes = slot_bytes;
  header_->overlay_data_offset = pixel_bytes;
  header_->encoding = config_.encoding;
  std::snprintf(header_->frame_id, sizeof(header_->frame_id), "%s",
                config_.frame_id.c_str());
  return true;
}

void DebugPreviewWriter::close() {
  if (mapping_) {
    ::munmap(mapping_, mapping_size_);
    mapping_ = nullptr;
  }
  mapping_size_ = 0;
  header_ = nullptr;
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  seq_ = 0;
}

bool DebugPreviewWriter::publish(const uint8_t* pixels, uint32_t width,
                                 uint32_t height, int64_t stamp_ns) {
  if (!header_ || !pixels || width == 0 || height == 0) {
    return false;
  }
  const uint32_t stride = width * 3u;
  const size_t bytes = static_cast<size_t>(stride) * height;
  if (bytes + sizeof(circle::debug::PreviewOverlayShmData) > header_->slot_bytes) {
    return false;
  }

  const uint32_t slot = static_cast<uint32_t>(seq_ % header_->slot_count);
  const size_t offset =
      sizeof(DebugPreviewShmHeader) +
      static_cast<size_t>(slot) * header_->slot_bytes;
  if (offset + bytes > mapping_size_) {
    return false;
  }

  auto* dst = reinterpret_cast<uint8_t*>(mapping_) + offset;
  std::memset(dst, 0, header_->slot_bytes);
  std::memcpy(dst, pixels, bytes);
  // Write zeroed overlay for backward compat
  auto* overlay_dst = dst + header_->overlay_data_offset;
  std::memset(overlay_dst, 0, sizeof(circle::debug::PreviewOverlayShmData));

  header_->width = width;
  header_->height = height;
  header_->stride = stride;
  header_->write_index = slot;
  header_->stamp_ns = stamp_ns;
  ++seq_;
  __atomic_store_n(&header_->seq, seq_, __ATOMIC_RELEASE);
  return true;
}

bool DebugPreviewWriter::publishWithOverlay(
    const uint8_t* pixels, uint32_t width, uint32_t height, int64_t stamp_ns,
    const circle::debug::PreviewOverlayShmData& overlay) {
  if (!header_ || !pixels || width == 0 || height == 0) {
    return false;
  }
  const uint32_t stride = width * 3u;
  const size_t pixel_bytes = static_cast<size_t>(stride) * height;
  const size_t overlay_bytes = sizeof(circle::debug::PreviewOverlayShmData);
  if (pixel_bytes + overlay_bytes > header_->slot_bytes) {
    return false;
  }

  const uint32_t slot = static_cast<uint32_t>(seq_ % header_->slot_count);
  const size_t offset =
      sizeof(DebugPreviewShmHeader) +
      static_cast<size_t>(slot) * header_->slot_bytes;
  if (offset + pixel_bytes + overlay_bytes > mapping_size_) {
    return false;
  }

  auto* dst = reinterpret_cast<uint8_t*>(mapping_) + offset;
  std::memset(dst, 0, header_->slot_bytes);
  std::memcpy(dst, pixels, pixel_bytes);
  std::memcpy(dst + header_->overlay_data_offset, &overlay, overlay_bytes);

  header_->width = width;
  header_->height = height;
  header_->stride = stride;
  header_->write_index = slot;
  header_->stamp_ns = stamp_ns;
  ++seq_;
  __atomic_store_n(&header_->seq, seq_, __ATOMIC_RELEASE);
  return true;
}

DebugPreviewReader::~DebugPreviewReader() { close(); }

bool DebugPreviewReader::open(const std::string& name) {
  close();
  fd_ = ::shm_open(name.c_str(), O_RDONLY, 0);
  if (fd_ < 0) {
    status_ = "shm_open failed: " + std::string(std::strerror(errno));
    return false;
  }
  struct stat st{};
  if (::fstat(fd_, &st) != 0 ||
      st.st_size <= static_cast<off_t>(sizeof(DebugPreviewShmHeader))) {
    status_ = "invalid shm size";
    close();
    return false;
  }
  mapping_size_ = static_cast<size_t>(st.st_size);
  mapping_ = ::mmap(nullptr, mapping_size_, PROT_READ, MAP_SHARED, fd_, 0);
  if (mapping_ == MAP_FAILED) {
    mapping_ = nullptr;
    status_ = std::string("mmap failed: ") + std::strerror(errno);
    close();
    return false;
  }
  header_ = reinterpret_cast<const DebugPreviewShmHeader*>(mapping_);
  if (std::memcmp(header_->magic, kMagic, sizeof(kMagic)) != 0 ||
      header_->version != kVersion || header_->slot_count == 0 ||
      header_->slot_bytes == 0) {
    status_ = "invalid header";
    close();
    return false;
  }
  status_ = "opened";
  return true;
}

void DebugPreviewReader::close() {
  if (mapping_) {
    ::munmap(mapping_, mapping_size_);
    mapping_ = nullptr;
  }
  mapping_size_ = 0;
  header_ = nullptr;
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool DebugPreviewReader::readLatest(DebugPreviewFrame& out) {
  if (!header_ || !mapping_) {
    return false;
  }

  for (int attempt = 0; attempt < 8; ++attempt) {
    const uint64_t seq_before =
        __atomic_load_n(&header_->seq, __ATOMIC_ACQUIRE);
    if (seq_before == 0) {
      last_seq_ = 0;
      return false;
    }
    if (last_seq_ != 0 && seq_before < last_seq_) {
      last_seq_ = 0;
    }
    if (seq_before == last_seq_) {
      return false;
    }

    const uint32_t slot_count = header_->slot_count;
    const uint32_t slot_bytes = header_->slot_bytes;
    const uint32_t write_index = header_->write_index;
    const uint32_t width = header_->width;
    const uint32_t height = header_->height;
    const uint32_t stride = header_->stride;
    const int64_t stamp_ns = header_->stamp_ns;

    if (slot_count == 0 || slot_bytes == 0 || width == 0 || height == 0 ||
        stride < width * 3u) {
      return false;
    }

    const uint32_t slot = write_index % slot_count;
    const size_t offset =
        sizeof(DebugPreviewShmHeader) +
        static_cast<size_t>(slot) * slot_bytes;
    if (offset + slot_bytes > mapping_size_) {
      return false;
    }

    const size_t row_bytes = static_cast<size_t>(width) * 3u;
    const size_t packed_bytes = row_bytes * height;
    if (packed_bytes == 0 || packed_bytes > slot_bytes) {
      return false;
    }

    const auto* src = reinterpret_cast<const uint8_t*>(mapping_) + offset;
    out.data.resize(packed_bytes);
    if (stride == row_bytes) {
      std::memcpy(out.data.data(), src, packed_bytes);
    } else {
      for (uint32_t row = 0; row < height; ++row) {
        std::memcpy(out.data.data() + static_cast<size_t>(row) * row_bytes,
                    src + static_cast<size_t>(row) * stride, row_bytes);
      }
    }

    const uint64_t seq_after =
        __atomic_load_n(&header_->seq, __ATOMIC_ACQUIRE);
    if (seq_before != seq_after) {
      continue;
    }

    out.seq = seq_before;
    out.stamp_ns = stamp_ns;
    out.width = width;
    out.height = height;
    out.stride = static_cast<uint32_t>(row_bytes);
    out.encoding = (header_->encoding == 2) ? "bgr8" : "rgb8";
    out.frame_id.assign(
        header_->frame_id,
        strnlen(header_->frame_id, sizeof(header_->frame_id)));
    last_seq_ = seq_before;
    return true;
  }
  return false;
}

bool DebugPreviewReader::readLatestWithOverlay(DebugPreviewFrameWithOverlay& out) {
  if (!header_ || !mapping_) {
    return false;
  }

  for (int attempt = 0; attempt < 8; ++attempt) {
    const uint64_t seq_before =
        __atomic_load_n(&header_->seq, __ATOMIC_ACQUIRE);
    if (seq_before == 0) {
      last_seq_ = 0;
      return false;
    }
    if (last_seq_ != 0 && seq_before < last_seq_) {
      last_seq_ = 0;
    }
    if (seq_before == last_seq_) {
      return false;
    }

    const uint32_t slot_count = header_->slot_count;
    const uint32_t slot_bytes = header_->slot_bytes;
    const uint32_t write_index = header_->write_index;
    const uint32_t width = header_->width;
    const uint32_t height = header_->height;
    const uint32_t stride = header_->stride;
    const int64_t stamp_ns = header_->stamp_ns;
    const uint32_t overlay_offset = header_->overlay_data_offset;

    if (slot_count == 0 || slot_bytes == 0 || width == 0 || height == 0 ||
        stride < width * 3u) {
      return false;
    }

    const uint32_t slot = write_index % slot_count;
    const size_t offset =
        sizeof(DebugPreviewShmHeader) +
        static_cast<size_t>(slot) * slot_bytes;
    if (offset + slot_bytes > mapping_size_) {
      return false;
    }

    const size_t row_bytes = static_cast<size_t>(width) * 3u;
    const size_t packed_bytes = row_bytes * height;
    if (packed_bytes == 0 || packed_bytes > slot_bytes) {
      return false;
    }

    const auto* src = reinterpret_cast<const uint8_t*>(mapping_) + offset;
    out.data.resize(packed_bytes);
    if (stride == row_bytes) {
      std::memcpy(out.data.data(), src, packed_bytes);
    } else {
      for (uint32_t row = 0; row < height; ++row) {
        std::memcpy(out.data.data() + static_cast<size_t>(row) * row_bytes,
                    src + static_cast<size_t>(row) * stride, row_bytes);
      }
    }

    // Read overlay data if present
    const size_t overlay_size = sizeof(circle::debug::PreviewOverlayShmData);
    if (overlay_offset > 0 &&
        overlay_offset + overlay_size <= slot_bytes) {
      std::memcpy(&out.overlay, src + overlay_offset, overlay_size);
      out.overlay_valid = true;
    } else {
      out.overlay = circle::debug::PreviewOverlayShmData{};
      out.overlay_valid = false;
    }

    const uint64_t seq_after =
        __atomic_load_n(&header_->seq, __ATOMIC_ACQUIRE);
    if (seq_before != seq_after) {
      continue;
    }

    out.seq = seq_before;
    out.stamp_ns = stamp_ns;
    out.width = width;
    out.height = height;
    out.stride = static_cast<uint32_t>(row_bytes);
    out.encoding = (header_->encoding == 2) ? "bgr8" : "rgb8";
    out.frame_id.assign(
        header_->frame_id,
        strnlen(header_->frame_id, sizeof(header_->frame_id)));
    last_seq_ = seq_before;
    return true;
  }
  return false;
}

}  // namespace circle::ipc
