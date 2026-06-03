#include "circle/bf/msp_client.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cerrno>
#include <cstring>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

namespace circle::bf {

namespace {

constexpr uint8_t kMspHeader = '$';
constexpr uint8_t kMspVersion = 'M';
constexpr uint8_t kMspRequest = '<';
constexpr uint8_t kMspResponse = '>';

uint8_t crc8(const uint8_t* data, size_t len) {
  uint8_t crc = 0;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
  }
  return crc;
}

speed_t baudToSpeed(int baudrate) {
  switch (baudrate) {
    case 115200:
      return B115200;
    case 921600:
      return B921600;
    default:
      return B115200;
  }
}

int16_t readLeI16(const uint8_t* p) {
  return static_cast<int16_t>(static_cast<uint16_t>(p[0]) |
                              (static_cast<uint16_t>(p[1]) << 8));
}

std::vector<uint8_t> buildRequestFrame(uint8_t cmd, const uint8_t* payload,
                                       uint8_t payload_len) {
  std::array<uint8_t, 5> header{};
  header[0] = kMspHeader;
  header[1] = kMspVersion;
  header[2] = kMspRequest;
  header[3] = payload_len;
  header[4] = cmd;
  std::vector<uint8_t> frame;
  frame.reserve(header.size() + payload_len + 1);
  frame.insert(frame.end(), header.begin(), header.end());
  if (payload_len > 0 && payload != nullptr) {
    frame.insert(frame.end(), payload, payload + payload_len);
  }
  frame.push_back(crc8(frame.data() + 3, frame.size() - 3));
  return frame;
}

/**
 * Pop the first complete, CRC-valid MSP response frame from the front of `buf`.
 * On success fills (cmd, payload) and erases consumed bytes. Returns false when
 * no full frame is available yet (caller should read more bytes).
 */
bool popResponseFrame(std::vector<uint8_t>& buf, uint8_t& out_cmd,
                      std::vector<uint8_t>& out_payload) {
  size_t off = 0;
  while (off + 6 <= buf.size()) {
    if (!(buf[off] == kMspHeader && buf[off + 1] == kMspVersion &&
          buf[off + 2] == kMspResponse)) {
      ++off;
      continue;
    }
    const uint8_t resp_len = buf[off + 3];
    const size_t frame_size = static_cast<size_t>(6) + resp_len;
    if (off + frame_size > buf.size()) {
      break;  // need more bytes
    }
    const uint8_t* frame = buf.data() + off;
    const uint8_t checksum = crc8(frame + 3, static_cast<size_t>(2 + resp_len));
    if (checksum != frame[5 + resp_len]) {
      ++off;  // bad CRC, resync
      continue;
    }
    out_cmd = frame[4];
    out_payload.assign(frame + 5, frame + 5 + resp_len);
    buf.erase(buf.begin(),
              buf.begin() + static_cast<std::ptrdiff_t>(off + frame_size));
    return true;
  }
  if (off > 0) {
    buf.erase(buf.begin(), buf.begin() + static_cast<std::ptrdiff_t>(off));
  }
  return false;
}

void parseStatusPayload(const std::vector<uint8_t>& resp, MspStatus& out) {
  out.cycle_time_us = static_cast<uint16_t>(resp[0] | (resp[1] << 8));
  out.i2c_errors = static_cast<uint16_t>(resp[2] | (resp[3] << 8));
  out.sensor_flags = static_cast<uint16_t>(resp[4] | (resp[5] << 8));
  out.mode_flags = static_cast<uint32_t>(resp[6] | (resp[7] << 8) |
                                         (resp[8] << 16) | (resp[9] << 24));
  out.armed = (out.mode_flags & (1U << 0)) != 0;
}

}  // namespace

MspClient::~MspClient() { close(); }

bool MspClient::openSerial(const std::string& device, int baudrate) {
  close();
  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    return false;
  }

  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    close();
    return false;
  }
  cfsetispeed(&tty, baudToSpeed(baudrate));
  cfsetospeed(&tty, baudToSpeed(baudrate));
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_lflag = 0;
  tty.c_oflag = 0;
  tty.c_iflag = 0;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 1;
  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    close();
    return false;
  }
  baud_ = baudrate;
  return true;
}

void MspClient::close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

void MspClient::setError(std::string msg) { last_error_ = std::move(msg); }

uint32_t MspClient::wireTimeUs(uint32_t bytes, int baud) {
  if (bytes == 0 || baud <= 0) {
    return 0;
  }
  // UART 8N1: 10 line bits per payload byte.
  return static_cast<uint32_t>(
      (static_cast<uint64_t>(bytes) * 10ULL * 1000000ULL) /
      static_cast<uint64_t>(baud));
}

void MspClient::recordIoSample(IoKind kind, uint32_t tx_bytes, uint32_t rx_bytes,
                               uint32_t round_trip_us, uint32_t wire_us,
                               uint32_t lock_wait_us, bool ok) {
  if (io_window_.window_start.time_since_epoch().count() == 0) {
    io_window_.window_start = std::chrono::steady_clock::now();
  }
  io_window_.tx_bytes += tx_bytes;
  io_window_.rx_bytes += rx_bytes;
  io_window_.lock_wait_us += lock_wait_us;
  io_window_.max_lock_wait_us =
      std::max(io_window_.max_lock_wait_us,
               static_cast<uint64_t>(lock_wait_us));
  if (kind == IoKind::kPoll) {
    if (ok) {
      ++io_window_.poll_ops;
    } else {
      ++io_window_.poll_fail;
    }
    io_window_.poll_round_trip_us += round_trip_us;
    io_window_.poll_wire_us += wire_us;
    io_window_.max_poll_round_trip_us =
        std::max(io_window_.max_poll_round_trip_us,
                 static_cast<uint64_t>(round_trip_us));
  } else {
    if (ok) {
      ++io_window_.send_ops;
    } else {
      ++io_window_.send_fail;
    }
    io_window_.send_round_trip_us += round_trip_us;
    io_window_.send_wire_us += wire_us;
    io_window_.max_send_round_trip_us =
        std::max(io_window_.max_send_round_trip_us,
                 static_cast<uint64_t>(round_trip_us));
  }
}

MspIoWindow MspClient::snapshotIoWindow() {
  std::lock_guard<std::mutex> io_lk(io_mu_);
  MspIoWindow out = io_window_;
  const auto now = std::chrono::steady_clock::now();
  if (out.window_start.time_since_epoch().count() == 0) {
    out.window_start = now;
  }
  out.window_s =
      std::chrono::duration<double>(now - out.window_start).count();
  io_window_ = MspIoWindow{};
  io_window_.window_start = now;
  return out;
}

bool MspClient::transact(MspCommand cmd,
                         const uint8_t* payload,
                         uint16_t payload_len,
                         std::vector<uint8_t>& response) {
  const auto t_before_lock = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> io_lk(io_mu_);
  const auto t_after_lock = std::chrono::steady_clock::now();
  const uint32_t lock_wait_us = static_cast<uint32_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(t_after_lock -
                                                            t_before_lock)
          .count());
  const auto t_start = t_after_lock;
  last_error_.clear();
  if (fd_ < 0) {
    setError("serial not open");
    recordIoSample(cmd == MspCommand::kSetRawRc ? IoKind::kSend : IoKind::kPoll,
                   0, 0, 0, 0, lock_wait_us, false);
    return false;
  }
  if (payload_len > 255 || static_cast<uint16_t>(cmd) > 255) {
    setError("payload/cmd out of range");
    recordIoSample(cmd == MspCommand::kSetRawRc ? IoKind::kSend : IoKind::kPoll,
                   0, 0, 0, 0, lock_wait_us, false);
    return false;
  }

  ::tcflush(fd_, TCIFLUSH);

  std::array<uint8_t, 5> header{};
  header[0] = kMspHeader;
  header[1] = kMspVersion;
  header[2] = kMspRequest;
  header[3] = static_cast<uint8_t>(payload_len);
  header[4] = static_cast<uint8_t>(static_cast<uint16_t>(cmd));

  std::vector<uint8_t> req_frame;
  req_frame.reserve(header.size() + payload_len + 1);
  req_frame.insert(req_frame.end(), header.begin(), header.end());
  if (payload_len > 0 && payload != nullptr) {
    req_frame.insert(req_frame.end(), payload, payload + payload_len);
  }
  req_frame.push_back(crc8(req_frame.data() + 3, req_frame.size() - 3));

  if (::write(fd_, req_frame.data(), req_frame.size()) !=
      static_cast<ssize_t>(req_frame.size())) {
    setError("serial write failed");
    recordIoSample(cmd == MspCommand::kSetRawRc ? IoKind::kSend : IoKind::kPoll,
                   static_cast<uint32_t>(req_frame.size()), 0, 0,
                   wireTimeUs(static_cast<uint32_t>(req_frame.size()), baud_),
                   lock_wait_us, false);
    return false;
  }

  std::vector<uint8_t> buf;
  buf.reserve(256);
  uint32_t rx_bytes = 0;
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(150);
  const uint8_t want_cmd = static_cast<uint8_t>(static_cast<uint16_t>(cmd));

  auto tryParseResponse = [&](std::vector<uint8_t>& response_out) -> bool {
    size_t off = 0;
    while (off + 6 <= buf.size()) {
      if (!(buf[off] == kMspHeader && buf[off + 1] == kMspVersion &&
            buf[off + 2] == kMspResponse)) {
        ++off;
        continue;
      }
      const uint8_t resp_len = buf[off + 3];
      const uint8_t resp_cmd = buf[off + 4];
      const size_t frame_size = static_cast<size_t>(6) + resp_len;
      if (off + frame_size > buf.size()) {
        break;
      }
      const uint8_t* resp_frame = buf.data() + off;
      const uint8_t checksum =
          crc8(resp_frame + 3, static_cast<size_t>(2 + resp_len));
      if (checksum != resp_frame[5 + resp_len]) {
        ++off;
        continue;
      }
      if (resp_cmd != want_cmd) {
        off += frame_size;
        continue;
      }
      response_out.assign(resp_frame + 5, resp_frame + 5 + resp_len);
      return true;
    }
    if (off > 0) {
      buf.erase(buf.begin(), buf.begin() + static_cast<std::ptrdiff_t>(off));
    }
    return false;
  };

  while (std::chrono::steady_clock::now() < deadline) {
    if (tryParseResponse(response)) {
      const auto t_end = std::chrono::steady_clock::now();
      const uint32_t round_trip_us = static_cast<uint32_t>(
          std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start)
              .count());
      const uint32_t tx_bytes = static_cast<uint32_t>(req_frame.size());
      const uint32_t wire_us =
          wireTimeUs(tx_bytes, baud_) + wireTimeUs(rx_bytes, baud_);
      recordIoSample(cmd == MspCommand::kSetRawRc ? IoKind::kSend : IoKind::kPoll,
                     tx_bytes, rx_bytes, round_trip_us, wire_us, lock_wait_us,
                     true);
      return true;
    }
    pollfd pfd{};
    pfd.fd = fd_;
    pfd.events = POLLIN;
    const int remaining_ms = static_cast<int>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now())
            .count());
    if (remaining_ms <= 0) {
      break;
    }
    if (::poll(&pfd, 1, std::max(1, remaining_ms)) <= 0) {
      continue;
    }
    std::array<uint8_t, 128> chunk{};
    const ssize_t n = ::read(fd_, chunk.data(), chunk.size());
    if (n > 0) {
      rx_bytes += static_cast<uint32_t>(n);
      buf.insert(buf.end(), chunk.data(), chunk.data() + n);
      if (buf.size() > 1024) {
        buf.erase(buf.begin(), buf.end() - 512);
      }
    }
  }

  if (tryParseResponse(response)) {
    const auto t_end = std::chrono::steady_clock::now();
    const uint32_t round_trip_us = static_cast<uint32_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start)
            .count());
    const uint32_t tx_bytes = static_cast<uint32_t>(req_frame.size());
    const uint32_t wire_us =
        wireTimeUs(tx_bytes, baud_) + wireTimeUs(rx_bytes, baud_);
    recordIoSample(cmd == MspCommand::kSetRawRc ? IoKind::kSend : IoKind::kPoll,
                   tx_bytes, rx_bytes, round_trip_us, wire_us, lock_wait_us,
                   true);
    return true;
  }

  setError(buf.empty()
               ? "timeout waiting for MSP response cmd=" +
                     std::to_string(static_cast<unsigned>(cmd))
               : "no valid MSP response cmd=" +
                     std::to_string(static_cast<unsigned>(cmd)) + " in " +
                     std::to_string(buf.size()) + " bytes");
  {
    const auto t_end = std::chrono::steady_clock::now();
    const uint32_t round_trip_us = static_cast<uint32_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start)
            .count());
    const uint32_t tx_bytes = static_cast<uint32_t>(req_frame.size());
    const uint32_t wire_us =
        wireTimeUs(tx_bytes, baud_) + wireTimeUs(rx_bytes, baud_);
    recordIoSample(cmd == MspCommand::kSetRawRc ? IoKind::kSend : IoKind::kPoll,
                   tx_bytes, rx_bytes, round_trip_us, wire_us, lock_wait_us,
                   false);
  }
  return false;
}

bool MspClient::readAttitude(MspAttitude& out) {
  std::vector<uint8_t> resp;
  if (!transact(MspCommand::kAttitude, nullptr, 0, resp) || resp.size() < 6) {
    return false;
  }
  out.roll_deci_deg = readLeI16(resp.data());
  out.pitch_deci_deg = readLeI16(resp.data() + 2);
  out.yaw_deci_deg = readLeI16(resp.data() + 4);
  return true;
}

bool MspClient::readStatus(MspStatus& out) {
  std::vector<uint8_t> resp;
  if (!transact(MspCommand::kStatus, nullptr, 0, resp) || resp.size() < 11) {
    return false;
  }
  // BF packFlightModeFlags(): BOXARM is the first active box → bit0 == ARMING_FLAG(ARMED).
  // Bit2 is typically HORIZON (or another mode), not armed — do not use for armed.
  parseStatusPayload(resp, out);
  return true;
}

bool MspClient::readRc(std::vector<uint16_t>* channels) {
  if (!channels) {
    setError("null channels out");
    return false;
  }
  std::vector<uint8_t> resp;
  if (!transact(MspCommand::kRc, nullptr, 0, resp)) {
    return false;
  }
  if (resp.size() < 2 || (resp.size() % 2) != 0) {
    setError("MSP_RC short/odd len=" + std::to_string(resp.size()));
    return false;
  }
  const size_t count = resp.size() / 2;
  channels->resize(count);
  for (size_t i = 0; i < count; ++i) {
    (*channels)[i] = static_cast<uint16_t>(
        resp[2 * i] | (static_cast<uint16_t>(resp[2 * i + 1]) << 8));
  }
  return true;
}

bool MspClient::readBoxIds(std::vector<uint8_t>* permanent_ids) {
  if (!permanent_ids) {
    setError("null permanent_ids out");
    return false;
  }
  std::vector<uint8_t> resp;
  if (!transact(MspCommand::kBoxIds, nullptr, 0, resp)) {
    return false;
  }
  if (resp.empty()) {
    setError("MSP_BOXIDS empty");
    return false;
  }
  *permanent_ids = std::move(resp);
  return true;
}

bool MspClient::sendRawRc(const std::vector<uint16_t>& channels) {
  std::vector<uint8_t> payload;
  payload.reserve(channels.size() * 2);
  for (uint16_t ch : channels) {
    payload.push_back(static_cast<uint8_t>(ch & 0xFF));
    payload.push_back(static_cast<uint8_t>((ch >> 8) & 0xFF));
  }
  std::vector<uint8_t> resp;
  return transact(MspCommand::kSetRawRc, payload.data(),
                  static_cast<uint16_t>(payload.size()), resp);
}

bool MspClient::writeFrameRaw(MspCommand cmd, const uint8_t* payload,
                              uint16_t len) {
  const auto t_before_lock = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> io_lk(io_mu_);
  const auto now = std::chrono::steady_clock::now();
  const uint32_t lock_wait_us = static_cast<uint32_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(now - t_before_lock)
          .count());
  last_error_.clear();
  if (fd_ < 0) {
    setError("serial not open");
    return false;
  }
  if (len > 255 || static_cast<uint16_t>(cmd) > 255) {
    setError("payload/cmd out of range");
    return false;
  }

  const auto frame = buildRequestFrame(
      static_cast<uint8_t>(static_cast<uint16_t>(cmd)), payload,
      static_cast<uint8_t>(len));
  const uint32_t tx_bytes = static_cast<uint32_t>(frame.size());
  const ssize_t n = ::write(fd_, frame.data(), frame.size());

  // Record the request timestamp so drainResponses can compute round-trip.
  switch (cmd) {
    case MspCommand::kSetRawRc:
      req_at_set_ = now;
      break;
    case MspCommand::kStatus:
      req_at_status_ = now;
      break;
    case MspCommand::kAttitude:
      req_at_attitude_ = now;
      break;
    case MspCommand::kRc:
      req_at_rc_ = now;
      break;
    default:
      break;
  }

  // Aggregate window stats: tx bytes + lock wait. SET writes count as send_ops
  // (command rate); poll-request responses are counted in drainResponses.
  if (io_window_.window_start.time_since_epoch().count() == 0) {
    io_window_.window_start = now;
  }
  io_window_.lock_wait_us += lock_wait_us;
  io_window_.max_lock_wait_us =
      std::max(io_window_.max_lock_wait_us, static_cast<uint64_t>(lock_wait_us));
  if (n != static_cast<ssize_t>(frame.size())) {
    setError("serial write failed");
    if (cmd == MspCommand::kSetRawRc) {
      ++io_window_.send_fail;
    } else {
      ++io_window_.poll_fail;
    }
    return false;
  }
  io_window_.tx_bytes += tx_bytes;
  if (cmd == MspCommand::kSetRawRc) {
    ++io_window_.send_ops;
  }
  return true;
}

bool MspClient::writeRawRc(const std::vector<uint16_t>& channels) {
  std::vector<uint8_t> payload;
  payload.reserve(channels.size() * 2);
  for (uint16_t ch : channels) {
    payload.push_back(static_cast<uint8_t>(ch & 0xFF));
    payload.push_back(static_cast<uint8_t>((ch >> 8) & 0xFF));
  }
  return writeFrameRaw(MspCommand::kSetRawRc, payload.data(),
                       static_cast<uint16_t>(payload.size()));
}

void MspClient::drainResponses(std::chrono::steady_clock::duration budget,
                               MspPollResult& out) {
  std::lock_guard<std::mutex> io_lk(io_mu_);
  if (fd_ < 0) {
    return;
  }
  const auto deadline = std::chrono::steady_clock::now() + budget;

  const auto handleFrame = [&](uint8_t cmd,
                               const std::vector<uint8_t>& payload,
                               size_t frame_bytes) {
    const auto now = std::chrono::steady_clock::now();
    const auto roundTripUs =
        [&](std::chrono::steady_clock::time_point req) -> uint32_t {
      if (req.time_since_epoch().count() == 0 || now < req) {
        return 0;
      }
      return static_cast<uint32_t>(
          std::chrono::duration_cast<std::chrono::microseconds>(now - req)
              .count());
    };
    if (cmd == static_cast<uint8_t>(MspCommand::kStatus)) {
      if (payload.size() >= 11) {
        parseStatusPayload(payload, out.status);
        out.status_ok = true;
        ++io_window_.poll_ops;
        const uint32_t rt = roundTripUs(req_at_status_);
        io_window_.poll_round_trip_us += rt;
        io_window_.poll_wire_us +=
            wireTimeUs(static_cast<uint32_t>(6 + frame_bytes), baud_);
        io_window_.max_poll_round_trip_us =
            std::max(io_window_.max_poll_round_trip_us,
                     static_cast<uint64_t>(rt));
      }
    } else if (cmd == static_cast<uint8_t>(MspCommand::kAttitude)) {
      if (payload.size() >= 6) {
        out.attitude.roll_deci_deg = readLeI16(payload.data());
        out.attitude.pitch_deci_deg = readLeI16(payload.data() + 2);
        out.attitude.yaw_deci_deg = readLeI16(payload.data() + 4);
        out.attitude_ok = true;
        ++io_window_.poll_ops;
        const uint32_t rt = roundTripUs(req_at_attitude_);
        io_window_.poll_round_trip_us += rt;
        io_window_.poll_wire_us +=
            wireTimeUs(static_cast<uint32_t>(6 + frame_bytes), baud_);
        io_window_.max_poll_round_trip_us =
            std::max(io_window_.max_poll_round_trip_us,
                     static_cast<uint64_t>(rt));
      }
    } else if (cmd == static_cast<uint8_t>(MspCommand::kRc)) {
      if (payload.size() >= 2 && (payload.size() % 2) == 0) {
        const size_t cnt = payload.size() / 2;
        out.rc.resize(cnt);
        for (size_t i = 0; i < cnt; ++i) {
          out.rc[i] = static_cast<uint16_t>(
              payload[2 * i] | (static_cast<uint16_t>(payload[2 * i + 1]) << 8));
        }
        out.rc_ok = true;
        ++io_window_.poll_ops;
        const uint32_t rt = roundTripUs(req_at_rc_);
        io_window_.poll_round_trip_us += rt;
        io_window_.poll_wire_us +=
            wireTimeUs(static_cast<uint32_t>(6 + frame_bytes), baud_);
        io_window_.max_poll_round_trip_us =
            std::max(io_window_.max_poll_round_trip_us,
                     static_cast<uint64_t>(rt));
      }
    } else if (cmd == static_cast<uint8_t>(MspCommand::kSetRawRc)) {
      // SET ack: account round-trip only (op already counted at write time).
      const uint32_t rt = roundTripUs(req_at_set_);
      io_window_.send_round_trip_us += rt;
      io_window_.send_wire_us +=
          wireTimeUs(static_cast<uint32_t>(6 + frame_bytes), baud_);
      io_window_.max_send_round_trip_us = std::max(
          io_window_.max_send_round_trip_us, static_cast<uint64_t>(rt));
    }
  };

  uint8_t cmd = 0;
  std::vector<uint8_t> payload;
  for (;;) {
    // Parse all complete frames already buffered.
    while (true) {
      const size_t before = drain_buf_.size();
      if (!popResponseFrame(drain_buf_, cmd, payload)) {
        break;
      }
      const size_t consumed = before - drain_buf_.size();
      handleFrame(cmd, payload, payload.size());
      io_window_.rx_bytes += consumed;
    }
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      break;
    }
    pollfd pfd{};
    pfd.fd = fd_;
    pfd.events = POLLIN;
    const int remaining_ms = static_cast<int>(
        std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now)
            .count());
    if (::poll(&pfd, 1, std::max(0, remaining_ms)) <= 0) {
      break;
    }
    std::array<uint8_t, 256> chunk{};
    const ssize_t n = ::read(fd_, chunk.data(), chunk.size());
    if (n <= 0) {
      break;
    }
    drain_buf_.insert(drain_buf_.end(), chunk.data(), chunk.data() + n);
    if (drain_buf_.size() > 2048) {
      drain_buf_.erase(drain_buf_.begin(), drain_buf_.end() - 1024);
    }
  }
}

bool MspClient::pollState(bool want_rc, MspPollResult& out) {
  const auto t_before_lock = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> io_lk(io_mu_);
  const auto t_after_lock = std::chrono::steady_clock::now();
  const uint32_t lock_wait_us = static_cast<uint32_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(t_after_lock -
                                                            t_before_lock)
          .count());
  const auto t_start = t_after_lock;
  out = MspPollResult{};
  out.status_requested = true;
  out.attitude_requested = true;
  out.rc_requested = want_rc;
  last_error_.clear();
  if (fd_ < 0) {
    setError("serial not open");
    recordIoSample(IoKind::kPoll, 0, 0, 0, 0, lock_wait_us, false);
    return false;
  }

  ::tcflush(fd_, TCIFLUSH);

  // Pipeline: write all requests back-to-back so BF queues them, then read the
  // batch of replies in one deadline window (one round-trip instead of N).
  std::vector<uint8_t> tx;
  const auto append = [&tx](const std::vector<uint8_t>& f) {
    tx.insert(tx.end(), f.begin(), f.end());
  };
  append(buildRequestFrame(static_cast<uint8_t>(MspCommand::kStatus), nullptr, 0));
  append(buildRequestFrame(static_cast<uint8_t>(MspCommand::kAttitude), nullptr, 0));
  if (want_rc) {
    append(buildRequestFrame(static_cast<uint8_t>(MspCommand::kRc), nullptr, 0));
  }
  const uint32_t tx_bytes = static_cast<uint32_t>(tx.size());
  if (::write(fd_, tx.data(), tx.size()) != static_cast<ssize_t>(tx.size())) {
    setError("serial write failed");
    recordIoSample(IoKind::kPoll, tx_bytes, 0, 0,
                   wireTimeUs(tx_bytes, baud_), lock_wait_us, false);
    return false;
  }

  const size_t want_count = want_rc ? 3 : 2;
  size_t got_count = 0;
  std::vector<uint8_t> buf;
  buf.reserve(256);
  uint32_t rx_bytes = 0;
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(150);

  const auto finishPoll = [&](bool ok) -> bool {
    const auto t_end = std::chrono::steady_clock::now();
    const uint32_t round_trip_us = static_cast<uint32_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start)
            .count());
    const uint32_t wire_us =
        wireTimeUs(tx_bytes, baud_) + wireTimeUs(rx_bytes, baud_);
    recordIoSample(IoKind::kPoll, tx_bytes, rx_bytes, round_trip_us, wire_us,
                   lock_wait_us, ok);
    return ok;
  };

  const auto dispatch = [&](uint8_t cmd, const std::vector<uint8_t>& payload) {
    if (cmd == static_cast<uint8_t>(MspCommand::kStatus)) {
      if (payload.size() >= 11) {
        parseStatusPayload(payload, out.status);
        out.status_ok = true;
        ++got_count;
      }
    } else if (cmd == static_cast<uint8_t>(MspCommand::kAttitude)) {
      if (payload.size() >= 6) {
        out.attitude.roll_deci_deg = readLeI16(payload.data());
        out.attitude.pitch_deci_deg = readLeI16(payload.data() + 2);
        out.attitude.yaw_deci_deg = readLeI16(payload.data() + 4);
        out.attitude_ok = true;
        ++got_count;
      }
    } else if (cmd == static_cast<uint8_t>(MspCommand::kRc)) {
      if (payload.size() >= 2 && (payload.size() % 2) == 0) {
        const size_t n = payload.size() / 2;
        out.rc.resize(n);
        for (size_t i = 0; i < n; ++i) {
          out.rc[i] = static_cast<uint16_t>(
              payload[2 * i] | (static_cast<uint16_t>(payload[2 * i + 1]) << 8));
        }
        out.rc_ok = true;
        ++got_count;
      }
    }
  };

  uint8_t cmd = 0;
  std::vector<uint8_t> payload;
  while (got_count < want_count &&
         std::chrono::steady_clock::now() < deadline) {
    while (popResponseFrame(buf, cmd, payload)) {
      dispatch(cmd, payload);
      if (got_count >= want_count) {
        break;
      }
    }
    if (got_count >= want_count) {
      break;
    }
    pollfd pfd{};
    pfd.fd = fd_;
    pfd.events = POLLIN;
    const int remaining_ms = static_cast<int>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now())
            .count());
    if (remaining_ms <= 0) {
      break;
    }
    if (::poll(&pfd, 1, std::max(1, remaining_ms)) <= 0) {
      continue;
    }
    std::array<uint8_t, 256> chunk{};
    const ssize_t n = ::read(fd_, chunk.data(), chunk.size());
    if (n > 0) {
      rx_bytes += static_cast<uint32_t>(n);
      buf.insert(buf.end(), chunk.data(), chunk.data() + n);
      if (buf.size() > 2048) {
        buf.erase(buf.begin(), buf.end() - 1024);
      }
    }
  }
  // Drain any remaining already-buffered frames (best-effort).
  while (popResponseFrame(buf, cmd, payload)) {
    dispatch(cmd, payload);
  }

  if (!out.status_ok || !out.attitude_ok) {
    setError("MSP poll incomplete (status_ok=" +
             std::to_string(out.status_ok ? 1 : 0) + " att_ok=" +
             std::to_string(out.attitude_ok ? 1 : 0) + " rc_ok=" +
             std::to_string(out.rc_ok ? 1 : 0) + ")");
    return finishPoll(false);
  }
  return finishPoll(true);
}

}  // namespace circle::bf
