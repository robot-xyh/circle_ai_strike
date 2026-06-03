#pragma once

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>

namespace circle::ipc {

class ParamBlockWriter {
 public:
  ~ParamBlockWriter();

  bool open(const std::string& name, size_t payload_capacity = 8192);
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

class ParamBlockReader {
 public:
  ~ParamBlockReader();

  bool open(const std::string& name);
  void close();
  bool readLatestJson(std::string& json_out);

 private:
  struct Header;

  int fd_{-1};
  void* mapping_{nullptr};
  size_t mapping_size_{0};
  const Header* header_{nullptr};
  const char* payload_{nullptr};
  uint64_t last_seq_{0};
};

}  // namespace circle::ipc
