#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "circle/debug/preview_overlay.hpp"

namespace circle::ipc {

/** Shared layout for debug preview SHM (writer + reader). */
struct DebugPreviewShmHeader {
  char magic[8];
  uint32_t version;
  uint32_t slot_count;
  uint32_t width;
  uint32_t height;
  uint32_t stride;
  uint32_t encoding;
  uint32_t slot_bytes;
  uint32_t overlay_data_offset;
  uint32_t write_index;
  uint64_t seq;
  int64_t stamp_ns;
  char frame_id[64];
};

struct DebugPreviewFrame {
  uint64_t seq{0};
  int64_t stamp_ns{0};
  uint32_t width{0};
  uint32_t height{0};
  uint32_t stride{0};
  std::string encoding;
  std::string frame_id;
  std::vector<uint8_t> data;
};

struct DebugPreviewFrameWithOverlay {
  uint64_t seq{0};
  int64_t stamp_ns{0};
  uint32_t width{0};
  uint32_t height{0};
  uint32_t stride{0};
  std::string encoding;
  std::string frame_id;
  std::vector<uint8_t> data;
  circle::debug::PreviewOverlayShmData overlay{};
  bool overlay_valid{false};
};

struct DebugPreviewWriterConfig {
  std::string shm_name;
  uint32_t width{0};
  uint32_t height{0};
  uint32_t slot_count{3};
  uint32_t encoding{2};  // 1=rgb8, 2=bgr8
  std::string frame_id{"bf_camera"};
};

class DebugPreviewWriter {
 public:
  DebugPreviewWriter() = default;
  ~DebugPreviewWriter();

  DebugPreviewWriter(const DebugPreviewWriter&) = delete;
  DebugPreviewWriter& operator=(const DebugPreviewWriter&) = delete;

  bool open(const DebugPreviewWriterConfig& config);
  void close();
  bool isOpen() const { return mapping_ != nullptr; }
  uint32_t width() const { return config_.width; }
  uint32_t height() const { return config_.height; }
  uint64_t sequence() const { return seq_; }

  /** Publish one BGR/RGB frame (row-major, stride = width * 3). */
  bool publish(const uint8_t* pixels, uint32_t width, uint32_t height,
               int64_t stamp_ns = 0);

  /** Publish one BGR/RGB frame with overlay context data. */
  bool publishWithOverlay(const uint8_t* pixels, uint32_t width, uint32_t height,
                          int64_t stamp_ns,
                          const circle::debug::PreviewOverlayShmData& overlay);

 private:
  int fd_{-1};
  void* mapping_{nullptr};
  size_t mapping_size_{0};
  DebugPreviewShmHeader* header_{nullptr};
  uint64_t seq_{0};
  DebugPreviewWriterConfig config_{};
};

class DebugPreviewReader {
 public:
  DebugPreviewReader() = default;
  ~DebugPreviewReader();

  DebugPreviewReader(const DebugPreviewReader&) = delete;
  DebugPreviewReader& operator=(const DebugPreviewReader&) = delete;

  bool open(const std::string& name);
  void close();
  bool isOpen() const { return mapping_ != nullptr; }
  std::string status() const { return status_; }
  bool readLatest(DebugPreviewFrame& out);

  bool readLatestWithOverlay(DebugPreviewFrameWithOverlay& out);

 private:
  int fd_{-1};
  void* mapping_{nullptr};
  size_t mapping_size_{0};
  const DebugPreviewShmHeader* header_{nullptr};
  std::string status_{"not opened"};
  uint64_t last_seq_{0};
};

}  // namespace circle::ipc
