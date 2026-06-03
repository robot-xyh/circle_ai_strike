#pragma once

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>

namespace circle::ipc {

/** bf_flight → bf_debugd: live strike-core params JSON for /api/params.json */
class StrikeParamsSnapshotWriter {
 public:
  ~StrikeParamsSnapshotWriter();
  bool open(const std::string& name, size_t payload_capacity = 32768);
  void close();
  bool writeJson(const std::string& json);

 private:
  struct Header;

  int fd_{-1};
  void* mapping_{nullptr};
  size_t mapping_size_{0};
  Header* header_{nullptr};
  char* payload_{nullptr};
  std::mutex mu_;
};

class StrikeParamsSnapshotReader {
 public:
  ~StrikeParamsSnapshotReader();
  bool open(const std::string& name);
  void close();
  /** Returns false when empty or SHM not open. */
  bool readCurrentJson(std::string& json_out) const;
  /** Monotonic publish counter; 0 when SHM is not open or never written. */
  uint64_t currentSeq() const;

 private:
  struct Header;

  int fd_{-1};
  void* mapping_{nullptr};
  size_t mapping_size_{0};
  const Header* header_{nullptr};
  const char* payload_{nullptr};
};

}  // namespace circle::ipc
