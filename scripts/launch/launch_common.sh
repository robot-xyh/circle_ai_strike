#!/usr/bin/env bash
# 供 scripts/launch 下各启动脚本 source 的公共定义。
# 使用前请在本脚本同目录下 source，例如: source "$(dirname "${BASH_SOURCE[0]}")/launch_common.sh"
# 提供: SCRIPT_DIR, WS_ROOT, setup_ros_ws, setup_logging, setup_performance

# 本文件所在目录即 launch 脚本所在目录；工作空间根目录为 circle_pilot 上两级（含 src 的 ws 根）
_LAUNCH_COMMON_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT_DIR="${_LAUNCH_COMMON_DIR}"
WS_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"

# 锁定 NPU / DDR / CPU 频率为 performance 模式（消除 DVFS 延迟尖峰）。
# 设 SKIP_PERFORMANCE=1 可跳过。
setup_performance() {
  if [[ "${SKIP_PERFORMANCE:-0}" = "1" ]]; then
    return 0
  fi
  local perf_script="${_LAUNCH_COMMON_DIR}/set_performance.sh"
  if [[ -x "${perf_script}" ]]; then
    source "${perf_script}"
  fi
}

# 加载 ROS 与工作空间环境。若 install 不存在则报错并 exit 1。
setup_ros_ws() {
  if [[ ! -f "${WS_ROOT}/install/setup.bash" ]]; then
    echo "错误: 未找到工作空间 install，请先在该工作空间下执行 colcon build。" >&2
    echo "  工作空间路径: ${WS_ROOT}" >&2
    exit 1
  fi
  source /opt/ros/humble/setup.bash
  export ROS_DOMAIN_ID=0
  source "${WS_ROOT}/install/setup.bash"

  # Rockchip gst-rockchip plugin (mpph264enc) loads librga.so.2; manual RGA installs
  # often live only under /usr/local/lib and are invisible to ROS nodes unless this is set.
  if [[ -e /usr/local/lib/librga.so.2 ]] || [[ -e /usr/local/lib/librga.so ]]; then
    if [[ ":${LD_LIBRARY_PATH:-}:" != *":/usr/local/lib:"* ]]; then
      export LD_LIBRARY_PATH="/usr/local/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
    fi
  fi
}

# 设置当前目录下的日志路径并创建 logs_ws 目录。
# 参数: 日志文件名前缀（不含日期与 .log），例如 "radar" -> logs_ws/radar_20250101_120000.log
# 设置变量: CWD, LOG_DIR, LOG_FILE
setup_logging() {
  local log_basename="${1:-launch}"
  CWD="$(pwd)"
  LOG_DIR="${CWD}/logs_ws"
  mkdir -p "${LOG_DIR}"
  LOG_FILE="${LOG_DIR}/${log_basename}_$(date +%Y%m%d_%H%M%S).log"
}
