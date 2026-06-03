#pragma once

#include <cstdint>

namespace circle::perception {

struct FramePipelineTiming {
  float wait_grab_ms{0.0F};
  float producer_ms{0.0F};
  float queue_wait_ms{0.0F};
  float cnn_ms{0.0F};
  float control_wait_ms{0.0F};
  float exec_ms{0.0F};
  float msp_gate_ms{0.0F};
  float e2e_wire_ms{0.0F};
  float e2e_algo_ms{0.0F};
  int64_t grab_done_ns{0};
  int64_t grab_start_steady_ns{0};
  int64_t grab_done_steady_ns{0};
  int64_t producer_done_steady_ns{0};
  int64_t infer_start_steady_ns{0};
  int64_t infer_done_steady_ns{0};
};

}  // namespace circle::perception
