#pragma once

#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <vector>

namespace circle::ipc {

constexpr uint32_t kZeroCopyMagic = 0x43505A43;  // "CPZC"
constexpr uint32_t kZeroCopyVersion = 2;
constexpr uint32_t kZeroCopyFrameFlagDebugPreviewWritten = 1u << 0u;

enum class ZeroCopyMsgType : uint32_t {
  Hello = 1,
  FrameReady = 2,
  FrameAck = 3,
};

struct ZeroCopyHello {
  uint32_t magic{kZeroCopyMagic};
  uint32_t version{kZeroCopyVersion};
  uint32_t type{static_cast<uint32_t>(ZeroCopyMsgType::Hello)};
  uint32_t model_width{0};
  uint32_t model_height{0};
  uint32_t input_buffer_size{0};
  uint32_t slot_count{1};
};

struct ZeroCopyFrameReady {
  uint32_t magic{kZeroCopyMagic};
  uint32_t version{kZeroCopyVersion};
  uint32_t type{static_cast<uint32_t>(ZeroCopyMsgType::FrameReady)};
  uint32_t width{0};
  uint32_t height{0};
  uint32_t step{0};
  uint32_t seq{0};
  uint32_t slot_id{0};
  int64_t stamp_sec{0};
  uint32_t stamp_nsec{0};
  uint32_t flags{0};
  uint64_t debug_preview_seq{0};
};

struct ZeroCopyFrameAck {
  uint32_t magic{kZeroCopyMagic};
  uint32_t version{kZeroCopyVersion};
  uint32_t type{static_cast<uint32_t>(ZeroCopyMsgType::FrameAck)};
  uint32_t seq{0};
  uint32_t slot_id{0};
};

inline bool sendAll(int fd, const void* data, size_t size) {
  const auto* p = static_cast<const uint8_t*>(data);
  while (size > 0) {
    const ssize_t n = send(fd, p, size, MSG_NOSIGNAL);
    if (n < 0) {
      if (errno == EINTR) continue;
      return false;
    }
    if (n == 0) return false;
    p += n;
    size -= static_cast<size_t>(n);
  }
  return true;
}

inline bool recvAll(int fd, void* data, size_t size, int flags = 0) {
  auto* p = static_cast<uint8_t*>(data);
  while (size > 0) {
    const ssize_t n = recv(fd, p, size, flags);
    if (n < 0) {
      if (errno == EINTR) continue;
      return false;
    }
    if (n == 0) return false;
    p += n;
    size -= static_cast<size_t>(n);
    flags = 0;
  }
  return true;
}

inline bool sendFdsWithHello(int sock, const std::vector<int>& fds,
                             const ZeroCopyHello& hello) {
  if (fds.empty()) return false;
  const size_t fd_bytes = sizeof(int) * fds.size();
  std::vector<char> control(CMSG_SPACE(fd_bytes));
  struct iovec iov = {};
  iov.iov_base = const_cast<ZeroCopyHello*>(&hello);
  iov.iov_len = sizeof(hello);

  struct msghdr msg = {};
  msg.msg_iov = &iov;
  msg.msg_iovlen = 1;
  msg.msg_control = control.data();
  msg.msg_controllen = control.size();

  struct cmsghdr* cmsg = CMSG_FIRSTHDR(&msg);
  cmsg->cmsg_level = SOL_SOCKET;
  cmsg->cmsg_type = SCM_RIGHTS;
  cmsg->cmsg_len = CMSG_LEN(fd_bytes);
  std::memcpy(CMSG_DATA(cmsg), fds.data(), fd_bytes);
  msg.msg_controllen = cmsg->cmsg_len;

  while (true) {
    const ssize_t n = sendmsg(sock, &msg, MSG_NOSIGNAL);
    if (n < 0 && errno == EINTR) continue;
    return n == static_cast<ssize_t>(sizeof(hello));
  }
}

inline bool sendFdWithHello(int sock, int fd_to_send,
                            const ZeroCopyHello& hello) {
  return sendFdsWithHello(sock, std::vector<int>{fd_to_send}, hello);
}

inline bool recvHelloWithFds(int sock, ZeroCopyHello* hello,
                             std::vector<int>* received_fds) {
  constexpr size_t kMaxFds = 8;
  char control[CMSG_SPACE(sizeof(int) * kMaxFds)] = {};
  struct iovec iov = {};
  iov.iov_base = hello;
  iov.iov_len = sizeof(*hello);

  struct msghdr msg = {};
  msg.msg_iov = &iov;
  msg.msg_iovlen = 1;
  msg.msg_control = control;
  msg.msg_controllen = sizeof(control);

  ssize_t n = 0;
  do {
    n = recvmsg(sock, &msg, 0);
  } while (n < 0 && errno == EINTR);
  if (n != static_cast<ssize_t>(sizeof(*hello))) {
    return false;
  }

  received_fds->clear();
  for (struct cmsghdr* cmsg = CMSG_FIRSTHDR(&msg); cmsg != nullptr;
       cmsg = CMSG_NXTHDR(&msg, cmsg)) {
    if (cmsg->cmsg_level == SOL_SOCKET && cmsg->cmsg_type == SCM_RIGHTS) {
      const size_t bytes =
          cmsg->cmsg_len > CMSG_LEN(0) ? cmsg->cmsg_len - CMSG_LEN(0) : 0;
      const size_t count = bytes / sizeof(int);
      const int* data = reinterpret_cast<const int*>(CMSG_DATA(cmsg));
      for (size_t i = 0; i < count && i < kMaxFds; ++i) {
        received_fds->push_back(data[i]);
      }
      break;
    }
  }
  return !received_fds->empty();
}

inline bool recvHelloWithFd(int sock, ZeroCopyHello* hello, int* received_fd) {
  std::vector<int> fds;
  if (!recvHelloWithFds(sock, hello, &fds)) return false;
  *received_fd = fds.front();
  for (size_t i = 1; i < fds.size(); ++i) close(fds[i]);
  return true;
}

inline bool isValidZeroCopyHeader(uint32_t magic, uint32_t version,
                                  uint32_t type) {
  return magic == kZeroCopyMagic && version == kZeroCopyVersion &&
         (type == static_cast<uint32_t>(ZeroCopyMsgType::Hello) ||
          type == static_cast<uint32_t>(ZeroCopyMsgType::FrameReady) ||
          type == static_cast<uint32_t>(ZeroCopyMsgType::FrameAck));
}

}  // namespace circle::ipc
