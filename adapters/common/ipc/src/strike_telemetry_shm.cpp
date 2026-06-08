#include "circle/ipc/strike_telemetry_shm.hpp"

#include <cerrno>
#include <cmath>
#include <cstring>
#include <limits>
#include <sstream>
#include <type_traits>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace circle::ipc {

namespace {
constexpr char kMagic[8] = {'C', 'P', 'S', 'T', 'R', 'K', 'E', '1'};
constexpr uint32_t kVersion = 5;

void appendJsonValue(std::ostringstream& oss, float v) {
  if (std::isfinite(v)) {
    oss << v;
  } else {
    oss << "null";
  }
}

template <typename T>
void appendJsonValue(std::ostringstream& oss, T v) {
  oss << v;
}
}  // namespace

struct StrikeTelemetryWriter::Header {
  char magic[8];
  uint32_t version;
  uint32_t capacity;
  uint64_t seq;
  uint64_t write_index;
};

struct StrikeTelemetryReader::Header {
  char magic[8];
  uint32_t version;
  uint32_t capacity;
  uint64_t seq;
  uint64_t write_index;
};

StrikeTelemetryWriter::~StrikeTelemetryWriter() { close(); }

bool StrikeTelemetryWriter::open(const std::string& name, size_t sample_capacity) {
  close();
  const size_t bytes =
      sizeof(Header) + sample_capacity * sizeof(StrikeTelemetrySample);
  fd_ = ::shm_open(name.c_str(), O_CREAT | O_RDWR, 0666);
  if (fd_ < 0) {
    return false;
  }
  if (ftruncate(fd_, static_cast<off_t>(bytes)) != 0) {
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
  samples_ = reinterpret_cast<StrikeTelemetrySample*>(
      reinterpret_cast<uint8_t*>(mapping_) + sizeof(Header));
  std::memcpy(header_->magic, kMagic, sizeof(kMagic));
  header_->version = kVersion;
  header_->capacity = static_cast<uint32_t>(sample_capacity);
  header_->seq = 0;
  header_->write_index = 0;
  return true;
}

void StrikeTelemetryWriter::close() {
  if (mapping_) {
    ::munmap(mapping_, mapping_size_);
    mapping_ = nullptr;
  }
  mapping_size_ = 0;
  header_ = nullptr;
  samples_ = nullptr;
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

void StrikeTelemetryWriter::publish(const StrikeTelemetrySample& sample) {
  if (!header_ || !samples_ || header_->capacity == 0) {
    return;
  }
  std::lock_guard<std::mutex> lk(mu_);
  const uint64_t idx = header_->write_index % header_->capacity;
  samples_[idx] = sample;
  samples_[idx].seq = header_->seq + 1;
  header_->write_index += 1;
  header_->seq = samples_[idx].seq;
}

StrikeTelemetryReader::~StrikeTelemetryReader() { close(); }

bool StrikeTelemetryReader::open(const std::string& name) {
  close();
  fd_ = ::shm_open(name.c_str(), O_RDONLY, 0);
  if (fd_ < 0) {
    return false;
  }
  struct stat st{};
  if (fstat(fd_, &st) != 0 || st.st_size < static_cast<off_t>(sizeof(Header))) {
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
  samples_ = reinterpret_cast<const StrikeTelemetrySample*>(
      reinterpret_cast<const uint8_t*>(mapping_) + sizeof(Header));
  if (std::memcmp(header_->magic, kMagic, sizeof(kMagic)) != 0 ||
      header_->version != kVersion || header_->capacity == 0) {
    close();
    return false;
  }
  return true;
}

void StrikeTelemetryReader::close() {
  if (mapping_) {
    ::munmap(mapping_, mapping_size_);
    mapping_ = nullptr;
  }
  mapping_size_ = 0;
  header_ = nullptr;
  samples_ = nullptr;
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool StrikeTelemetryReader::readLatest(StrikeTelemetrySample& out) {
  if (!header_ || !samples_) {
    return false;
  }
  const uint64_t seq = header_->seq;
  if (seq == 0 || seq == last_seq_) {
    return false;
  }
  const uint64_t idx = (header_->write_index == 0 ? 0 : header_->write_index - 1) %
                       header_->capacity;
  out = samples_[idx];
  last_seq_ = seq;
  return true;
}

std::string StrikeTelemetryReader::seriesJson() const {
  if (!header_ || !samples_ || header_->capacity == 0) {
    return R"({"ok":true,"tune_mode":"target_strike","strike_backend":"bf","n":0,"t":[]})";
  }
  std::ostringstream oss;
  const uint32_t n = std::min<uint32_t>(header_->capacity, 256);
  const uint64_t start =
      header_->seq > n ? header_->write_index - n : 0;
  std::vector<StrikeTelemetrySample> ring;
  for (uint64_t i = start; i < header_->write_index; ++i) {
    const auto& s = samples_[i % header_->capacity];
    if (s.seq == 0) {
      continue;
    }
    ring.push_back(s);
  }
  const auto emitArray = [&](const char* name, auto getter) {
    oss << R"(,")" << name << R"(":[)";
    for (size_t i = 0; i < ring.size(); ++i) {
      if (i > 0) {
        oss << ',';
      }
      appendJsonValue(oss, getter(ring[i]));
    }
    oss << ']';
  };

  const bool is_png = !ring.empty() && ring.back().controller_kind == 1;
  const char* tune_mode = is_png ? "target_strike_png" : "target_strike";
  oss << R"({"ok":true,"tune_mode":")" << tune_mode
      << R"(","strike_backend":"bf","n":)" << ring.size();
  oss << R"(,"t":[)";
  for (size_t i = 0; i < ring.size(); ++i) {
    if (i > 0) {
      oss << ',';
    }
    oss << (static_cast<double>(ring[i].stamp_ns) * 1.0e-9);
  }
  oss << ']';
  emitArray("vision_ex", [](const auto& s) { return s.ex; });
  emitArray("vision_ey", [](const auto& s) { return s.ey; });
  emitArray("vision_roll_sp_rad", [](const auto& s) { return s.roll_rate_sp; });
  emitArray("vision_pitch_sp_rad", [](const auto& s) { return s.pitch_rate_sp; });
  emitArray("vision_yaw_rate_sp_rad_s", [](const auto& s) { return s.yaw_rate_sp; });
  emitArray("vision_thrust_z", [](const auto& s) { return s.thrust_z; });
  emitArray("vehicle_throttle_algo_norm",
            [](const auto& s) { return s.throttle_algo_norm; });
  emitArray("vehicle_throttle_cmd_norm", [](const auto& s) {
    return s.throttle_cmd_valid ? s.throttle_cmd_norm
                                : std::numeric_limits<float>::quiet_NaN();
  });
  emitArray("vehicle_roll_rad", [](const auto& s) {
    return s.vehicle_valid ? s.vehicle_roll_rad
                           : std::numeric_limits<float>::quiet_NaN();
  });
  emitArray("vehicle_pitch_rad", [](const auto& s) {
    return s.vehicle_valid ? s.vehicle_pitch_rad
                           : std::numeric_limits<float>::quiet_NaN();
  });
  emitArray("vehicle_yaw_rad", [](const auto& s) {
    return s.vehicle_valid ? s.vehicle_yaw_rad
                           : std::numeric_limits<float>::quiet_NaN();
  });
  emitArray("vehicle_throttle_pwm", [](const auto& s) {
    return s.vehicle_valid ? s.throttle_pwm
                           : std::numeric_limits<float>::quiet_NaN();
  });
  emitArray("vehicle_throttle_norm", [](const auto& s) {
    return s.vehicle_valid ? s.throttle_norm
                           : std::numeric_limits<float>::quiet_NaN();
  });
  emitArray("track_phase", [](const auto& s) { return s.state; });
  emitArray("has_target", [](const auto& s) { return static_cast<int>(s.has_target); });
  emitArray("armed", [](const auto& s) { return static_cast<int>(s.armed); });
  emitArray("dry_run_passthrough",
            [](const auto& s) { return static_cast<int>(s.dry_run_passthrough); });

  if (is_png) {
    constexpr float kNan = std::numeric_limits<float>::quiet_NaN();
    // BF has no measured body rates (BfStateSource fills attitude only), so the
    // actual-rate traces are always null on this (BF-only) path; the frontend
    // hides them automatically. PX4 fills these on its own (ROS) series path.
    emitArray("vehicle_roll_rate_rad_s", [&](const auto&) { return kNan; });
    emitArray("vehicle_pitch_rate_rad_s", [&](const auto&) { return kNan; });
    emitArray("vehicle_yaw_rate_rad_s", [&](const auto&) { return kNan; });

    emitArray("png_closure_scale", [](const auto& s) { return s.png_closure_scale; });
    emitArray("png_ex_dot_inertial",
              [](const auto& s) { return s.png_ex_dot_inertial; });
    emitArray("png_ey_dot_inertial",
              [](const auto& s) { return s.png_ey_dot_inertial; });
    emitArray("png_measurement_age_s",
              [](const auto& s) { return s.png_measurement_age_s; });
    emitArray("png_ff_roll_rad_s", [](const auto& s) { return s.png_ff_roll_rad_s; });
    emitArray("png_ff_pitch_rad_s",
              [](const auto& s) { return s.png_ff_pitch_rad_s; });
    emitArray("png_fov_trim_roll_rad_s",
              [](const auto& s) { return s.png_fov_trim_roll_rad_s; });
    emitArray("png_fov_trim_pitch_rad_s",
              [](const auto& s) { return s.png_fov_trim_pitch_rad_s; });
    emitArray("png_edge_guard_roll_rad_s",
              [](const auto& s) { return s.png_edge_guard_roll_rad_s; });
    emitArray("png_edge_guard_pitch_rad_s",
              [](const auto& s) { return s.png_edge_guard_pitch_rad_s; });
    emitArray("png_pursuit_roll_rad_s",
              [](const auto& s) { return s.png_pursuit_roll_rad_s; });
    emitArray("png_pursuit_pitch_rad_s",
              [](const auto& s) { return s.png_pursuit_pitch_rad_s; });
    emitArray("png_stale_trim_roll_rad_s",
              [](const auto& s) { return s.png_stale_trim_roll_rad_s; });
    emitArray("png_intercept_roll_rad_s",
              [](const auto& s) { return s.png_intercept_roll_rad_s; });
    emitArray("png_intercept_pitch_rad_s",
              [](const auto& s) { return s.png_intercept_pitch_rad_s; });
    emitArray("png_crossing_pitch_rad_s",
              [](const auto& s) { return s.png_crossing_pitch_rad_s; });
    emitArray("png_future_ex", [](const auto& s) { return s.png_future_ex; });
    emitArray("png_future_ey", [](const auto& s) { return s.png_future_ey; });
    emitArray("png_intercept_lead_s",
              [](const auto& s) { return s.png_intercept_lead_s; });
    emitArray("png_crossing_weight",
              [](const auto& s) { return s.png_crossing_weight; });
    emitArray("png_fwd_guard_scale",
              [](const auto& s) { return s.png_fwd_guard_scale; });
    emitArray("png_entry_handoff_progress",
              [](const auto& s) { return s.png_entry_handoff_progress; });
    emitArray("png_tilt_softcap_roll",
              [](const auto& s) { return s.png_tilt_softcap_roll; });
    emitArray("png_tilt_softcap_pitch",
              [](const auto& s) { return s.png_tilt_softcap_pitch; });
    emitArray("derotate_lookup_age_ms",
              [](const auto& s) { return s.png_derotate_lookup_age_ms; });
    emitArray("derotate_interp_gap_ms",
              [](const auto& s) { return s.png_derotate_interp_gap_ms; });
    emitArray("derotate_roll_rate_rad_s", [&](const auto& s) {
      return s.png_derotate_lookup_valid ? s.png_derotate_roll_rate_rad_s : kNan;
    });
    emitArray("derotate_pitch_rate_rad_s", [&](const auto& s) {
      return s.png_derotate_lookup_valid ? s.png_derotate_pitch_rate_rad_s : kNan;
    });
    emitArray("camera_exposure_midpoint_offset_ns", [](const auto& s) {
      return s.png_camera_exposure_midpoint_offset_ns;
    });
    emitArray("fc_serial_latency_ns",
              [](const auto& s) { return s.png_fc_serial_latency_ns; });
    emitArray("derotate_lookup_valid", [](const auto& s) {
      return static_cast<int>(s.png_derotate_lookup_valid);
    });
    emitArray("png_intercept_active", [](const auto& s) {
      return static_cast<int>(s.png_intercept_active);
    });
    emitArray("png_crossing_active", [](const auto& s) {
      return static_cast<int>(s.png_crossing_active);
    });
    emitArray("png_fwd_guard_active", [](const auto& s) {
      return static_cast<int>(s.png_fwd_guard_active);
    });
    emitArray("png_loss_hold_latched", [](const auto& s) {
      return static_cast<int>(s.png_loss_hold_latched);
    });
    emitArray("png_tilt_hardcap_active", [](const auto& s) {
      return static_cast<int>(s.png_tilt_hardcap_active);
    });
    emitArray("bbox_area_ratio", [](const auto& s) { return s.bbox_area_ratio; });
    emitArray("detection_score", [](const auto& s) { return s.detection_score; });
    emitArray("detection_valid", [](const auto& s) {
      return static_cast<int>(s.detection_valid);
    });
    emitArray("msp_override_active", [](const auto& s) {
      return static_cast<int>(s.msp_override_active);
    });
  }
  oss << '}';
  return oss.str();
}

}  // namespace circle::ipc
