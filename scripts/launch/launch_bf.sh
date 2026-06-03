#!/usr/bin/env bash
# 启动 Betaflight 双进程栈：bf_flight（主控）+ bf_debugd（调试/预览）。
#
# Usage:
#   ./launch_bf.sh                         # 默认同时启动主控与 debug
#   ./launch_bf.sh --flight                # 仅 bf_flight
#   ./launch_bf.sh --debug                 # 仅 bf_debugd
#   ./launch_bf.sh --flight-config PATH    # 覆盖主控配置
#   ./launch_bf.sh --debug-config PATH     # 覆盖 debug 配置
#   SKIP_PERFORMANCE=1 ./launch_bf.sh      # 跳过 performance 调频
#
# 日志写入 ${WS_ROOT}/logs_ws/<控制模式>_YYYYMMDD_HHMMSS.log
#   strike:     bf_flight_strike_*.log / bf_debugd_strike_*.log
#   strike_png: bf_flight_png_*.log   / bf_debugd_strike_png_*.log

set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/launch_common.sh"
setup_performance

MODE="both"
FLIGHT_CONFIG=""
DEBUG_CONFIG=""
# 0 = StrikeController (bf_flight)；1 = PNG 比例导引 (bf_flight_png)。
# bf_flight 与 bf_flight_png 共享串口/SHM，运行时互斥，只能二选一。
PNG=1

usage() {
  cat <<'EOF'
Usage: launch_bf.sh [OPTIONS]

默认同时启动 bf_flight 与 bf_debugd。

Options:
  --flight, --flight-only    仅启动主控
  --debug, --debug-only      仅启动 bf_debugd（调试/预览）
  --png                      使用 bf_flight_png（PNG 比例导引）替代 bf_flight，
                             并默认切到 strike_png_bf_flight/strike_png_bf_debug 配置
  --flight-config PATH       主控 YAML（默认随 --png 选择对应配置）
  --debug-config PATH        debug YAML（默认随 --png 选择对应配置）
  -h, --help                 显示本帮助

注意：bf_flight 与 bf_flight_png 互斥（共享 MSP 串口与 SHM），不可同时运行。

Environment:
  SKIP_PERFORMANCE=1         跳过 set_performance.sh
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --flight|--flight-only)
      MODE="flight"
      shift
      ;;
    --debug|--debug-only)
      MODE="debug"
      shift
      ;;
    --png)
      PNG=1
      shift
      ;;
    --flight-config)
      FLIGHT_CONFIG="${2:?missing path for --flight-config}"
      shift 2
      ;;
    --debug-config)
      DEBUG_CONFIG="${2:?missing path for --debug-config}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "未知参数: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
done

CIRCLE_PILOT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
BF_BUILD="${CIRCLE_PILOT_ROOT}/adapters/bf/build"
DEBUG_BIN="${BF_BUILD}/bf_debugd"
if [[ "${PNG}" -eq 1 ]]; then
  FLIGHT_BIN="${BF_BUILD}/bf_flight_png"
  FLIGHT_CONFIG="${FLIGHT_CONFIG:-${CIRCLE_PILOT_ROOT}/config/strike_png_bf_flight.yaml}"
  DEBUG_CONFIG="${DEBUG_CONFIG:-${CIRCLE_PILOT_ROOT}/config/strike_png_bf_debug.yaml}"
  FLIGHT_LOG_TAG="bf_flight_png"
  DEBUG_LOG_TAG="bf_debugd_strike_png"
  FLIGHT_PROC_NAME="bf_flight_png"
else
  FLIGHT_BIN="${BF_BUILD}/bf_flight"
  FLIGHT_CONFIG="${FLIGHT_CONFIG:-${CIRCLE_PILOT_ROOT}/config/strike_bf_flight.yaml}"
  DEBUG_CONFIG="${DEBUG_CONFIG:-${CIRCLE_PILOT_ROOT}/config/strike_bf_debug.yaml}"
  FLIGHT_LOG_TAG="bf_flight_strike"
  DEBUG_LOG_TAG="bf_debugd_strike"
  FLIGHT_PROC_NAME="bf_flight"
fi

LOG_DIR="${WS_ROOT}/logs_ws"
mkdir -p "${LOG_DIR}"
RUN_STAMP="$(date +%Y%m%d_%H%M%S)"
FLIGHT_LOG="${LOG_DIR}/${FLIGHT_LOG_TAG}_${RUN_STAMP}.log"
DEBUG_LOG="${LOG_DIR}/${DEBUG_LOG_TAG}_${RUN_STAMP}.log"

if [[ -e /usr/local/lib/librga.so.2 ]] || [[ -e /usr/local/lib/librga.so ]]; then
  if [[ ":${LD_LIBRARY_PATH:-}:" != *":/usr/local/lib:"* ]]; then
    export LD_LIBRARY_PATH="/usr/local/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
  fi
fi

require_binary() {
  local bin="$1"
  local hint="$2"
  if [[ ! -x "${bin}" ]]; then
    echo "错误: 未找到可执行文件 ${bin}" >&2
    echo "  请先编译: cd ${CIRCLE_PILOT_ROOT}/adapters/bf/build && cmake --build . -j" >&2
    echo "  ${hint}" >&2
    exit 1
  fi
}

require_config() {
  local cfg="$1"
  local name="$2"
  if [[ ! -f "${cfg}" ]]; then
    echo "错误: ${name} 配置文件不存在: ${cfg}" >&2
    exit 1
  fi
}

PIDS=()

cleanup() {
  local pid
  for pid in "${PIDS[@]}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill -TERM "${pid}" 2>/dev/null || true
    fi
  done
  for pid in "${PIDS[@]}"; do
    wait "${pid}" 2>/dev/null || true
  done
}

on_signal() {
  echo ""
  echo "收到退出信号，正在停止 BF 进程..."
  cleanup
  exit 130
}

trap on_signal INT TERM
trap cleanup EXIT

cd "${CIRCLE_PILOT_ROOT}"

start_flight() {
  require_binary "${FLIGHT_BIN}" "${FLIGHT_PROC_NAME}"
  require_config "${FLIGHT_CONFIG}" "${FLIGHT_PROC_NAME}"
  echo "启动 ${FLIGHT_PROC_NAME}"
  echo "  bin:    ${FLIGHT_BIN}"
  echo "  config: ${FLIGHT_CONFIG}"
  echo "  log:    ${FLIGHT_LOG}"
  "${FLIGHT_BIN}" -c "${FLIGHT_CONFIG}" >> "${FLIGHT_LOG}" 2>&1 &
  PIDS+=("$!")
}

start_debug() {
  require_binary "${DEBUG_BIN}" "bf_debugd"
  require_config "${DEBUG_CONFIG}" "bf_debugd"
  echo "启动 bf_debugd (${DEBUG_LOG_TAG})"
  echo "  bin:    ${DEBUG_BIN}"
  echo "  config: ${DEBUG_CONFIG}"
  echo "  log:    ${DEBUG_LOG}"
  "${DEBUG_BIN}" -c "${DEBUG_CONFIG}" >> "${DEBUG_LOG}" 2>&1 &
  PIDS+=("$!")
}

case "${MODE}" in
  flight)
    start_flight
    ;;
  debug)
    start_debug
    ;;
  both)
    start_flight
    start_debug
    ;;
  *)
    echo "内部错误: 未知 MODE=${MODE}" >&2
    exit 1
    ;;
esac

echo ""
echo "circle_pilot: ${CIRCLE_PILOT_ROOT}"
echo "工作目录:     $(pwd)"
echo "日志目录:     ${LOG_DIR}"
if [[ "${MODE}" == "flight" || "${MODE}" == "both" ]]; then
  echo "${FLIGHT_PROC_NAME} 日志: ${FLIGHT_LOG}"
fi
if [[ "${MODE}" == "debug" || "${MODE}" == "both" ]]; then
  echo "bf_debugd 日志: ${DEBUG_LOG}"
fi
echo "控制模式: ${FLIGHT_LOG_TAG#bf_flight_}  运行模式: ${MODE}  进程数: ${#PIDS[@]}"
echo "按 Ctrl+C 停止"
echo ""

if [[ ${#PIDS[@]} -eq 1 ]]; then
  wait "${PIDS[0]}"
  exit $?
fi

# 双进程：任一退出则结束另一个
while true; do
  for pid in "${PIDS[@]}"; do
    if ! kill -0 "${pid}" 2>/dev/null; then
      wait "${pid}" || true
      echo "进程 ${pid} 已退出，停止其余 BF 进程" >&2
      exit 1
    fi
  done
  sleep 0.5
done
