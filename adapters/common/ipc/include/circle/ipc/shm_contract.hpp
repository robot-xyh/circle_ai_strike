#pragma once

#include <cstdint>
#include <string>

namespace circle::ipc {

// POSIX shm_open names must begin with "/" and must not contain another slash.
constexpr const char* kStrikeTelemetryShmName = "/circle_pilot_strike_telemetry";
constexpr const char* kDebugPreviewShmName = "/circle_pilot_debug_preview";
constexpr const char* kParamBlockShmName = "/circle_pilot_param_block";
constexpr const char* kStrikeParamsSnapshotShmName =
    "/circle_pilot_strike_params_snapshot";

struct ShmHeader {
  uint64_t seq{0};
  uint64_t stamp_ns{0};
  uint32_t payload_bytes{0};
  uint32_t reserved{0};
};

struct ParamBlockHeader {
  uint64_t seq{0};
  uint32_t payload_bytes{0};
  uint32_t reserved{0};
};

}  // namespace circle::ipc
