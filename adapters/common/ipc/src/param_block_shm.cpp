#include "circle/ipc/param_block_shm.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace circle::ipc {

namespace {
constexpr char kMagic[8] = {'C', 'P', 'P', 'A', 'R', 'A', 'M', '1'};
constexpr uint32_t kVersion = 1;
}  // namespace

struct ParamBlockWriter::Header {
  char magic[8];
  uint32_t version;
  uint32_t capacity;
  uint32_t payload_bytes;
  uint32_t reserved;
  uint64_t seq;
};

struct ParamBlockReader::Header {
  char magic[8];
  uint32_t version;
  uint32_t capacity;
  uint32_t payload_bytes;
  uint32_t reserved;
  uint64_t seq;
};

ParamBlockWriter::~ParamBlockWriter() { close(); }

bool ParamBlockWriter::open(const std::string& name, size_t payload_capacity) {
  close();
  const size_t bytes = sizeof(Header) + payload_capacity;
  fd_ = ::shm_open(name.c_str(), O_CREAT | O_RDWR, 0666);
  if (fd_ < 0) {
    return false;
  }
  if (::ftruncate(fd_, static_cast<off_t>(bytes)) != 0) {
    close();
    return false;
  }
  mapping_size_ = bytes;
  mapping_ = ::mmap(nullptr, mapping_size_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
  if (mapping_ == MAP_FAILED) {
    mapping_ = nullptr;
    close();
    return false;
  }
  header_ = reinterpret_cast<Header*>(mapping_);
  payload_ = reinterpret_cast<char*>(mapping_) + sizeof(Header);
  std::memcpy(header_->magic, kMagic, sizeof(kMagic));
  header_->version = kVersion;
  header_->capacity = static_cast<uint32_t>(payload_capacity);
  header_->payload_bytes = 0;
  header_->seq = 0;
  return true;
}

void ParamBlockWriter::close() {
  if (mapping_) {
    ::munmap(mapping_, mapping_size_);
    mapping_ = nullptr;
  }
  mapping_size_ = 0;
  header_ = nullptr;
  payload_ = nullptr;
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool ParamBlockWriter::writeJson(const std::string& json) {
  if (!header_ || !payload_ || json.size() + 1 > header_->capacity) {
    return false;
  }
  std::lock_guard<std::mutex> lk(mu_);
  std::memcpy(payload_, json.data(), json.size());
  payload_[json.size()] = '\0';
  header_->payload_bytes = static_cast<uint32_t>(json.size());
  header_->seq += 1;
  return true;
}

ParamBlockReader::~ParamBlockReader() { close(); }

bool ParamBlockReader::open(const std::string& name) {
  close();
  fd_ = ::shm_open(name.c_str(), O_RDONLY, 0);
  if (fd_ < 0) {
    return false;
  }
  struct stat st {};
  if (::fstat(fd_, &st) != 0 || st.st_size <= static_cast<off_t>(sizeof(Header))) {
    close();
    return false;
  }
  mapping_size_ = static_cast<size_t>(st.st_size);
  mapping_ = ::mmap(nullptr, mapping_size_, PROT_READ, MAP_SHARED, fd_, 0);
  if (mapping_ == MAP_FAILED) {
    mapping_ = nullptr;
    close();
    return false;
  }
  header_ = reinterpret_cast<const Header*>(mapping_);
  payload_ = reinterpret_cast<const char*>(mapping_) + sizeof(Header);
  if (std::memcmp(header_->magic, kMagic, sizeof(kMagic)) != 0 ||
      header_->version != kVersion || header_->capacity == 0) {
    close();
    return false;
  }
  return true;
}

void ParamBlockReader::close() {
  if (mapping_) {
    ::munmap(mapping_, mapping_size_);
    mapping_ = nullptr;
  }
  mapping_size_ = 0;
  header_ = nullptr;
  payload_ = nullptr;
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool ParamBlockReader::readLatestJson(std::string& json_out) {
  if (!header_ || !payload_ || header_->seq == 0 || header_->seq == last_seq_) {
    return false;
  }
  const uint64_t seq_before = header_->seq;
  const uint32_t n = std::min(header_->payload_bytes, header_->capacity);
  json_out.assign(payload_, payload_ + n);
  if (header_->seq != seq_before) {
    return false;
  }
  last_seq_ = seq_before;
  return true;
}

}  // namespace circle::ipc
