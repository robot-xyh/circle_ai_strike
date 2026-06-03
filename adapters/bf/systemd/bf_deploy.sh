#!/usr/bin/env bash
# Betaflight 一体化部署工具：编译 + 打包 + 安装 systemd 服务。
#
# 把 bf_flight / bf_debugd 及其运行所需资源（配置、相机内参、模型、web 资源）
# 打包成一个自包含目录（PREFIX），并渲染 systemd 单元安装到用户目录
# （systemctl --user，默认）或系统目录（systemctl，需 root）。
#
# 打包后的 PREFIX 目录镜像 circle_pilot 的相对路径布局，因此配置内的相对路径
# （camera / camera_info / plots_html / strike_flight_config）无需改写即可解析；
# model_path 会被复制进 PREFIX 并改写为相对路径，使整个包可独立运行。
#
# 用法:
#   ./bf_deploy.sh all                 # 编译 + 打包 + 安装 + enable + start（用户级，推荐）
#   ./bf_deploy.sh build               # 仅编译
#   ./bf_deploy.sh package             # 仅打包到 PREFIX
#   ./bf_deploy.sh install             # 打包 + 安装单元（不 enable/start）
#   ./bf_deploy.sh enable              # install + 开机自启
#   ./bf_deploy.sh start               # install + enable + 立即启动
#   ./bf_deploy.sh stop|disable|status
#   ./bf_deploy.sh uninstall           # 停止/禁用/删除单元（加 --purge 连同 PREFIX 一起删）
#
# 选项:
#   --user            用户级 systemd（默认，PREFIX=~/.local/share/circle_bf）
#   --system          系统级 systemd（需 root，PREFIX=/opt/circle）
#   --prefix DIR      覆盖打包/安装目录
#   --jobs N          编译并行数（默认 4）
#   --flight-config F 覆盖源 bf_flight 配置（默认 config/strike_bf_flight.yaml）
#   --debug-config F  覆盖源 bf_debugd 配置（默认 config/strike_bf_debug.yaml）
#   --purge           uninstall 时一并删除 PREFIX
#   -h, --help

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 本文件位于 .../circle_pilot/adapters/bf/systemd
PKG_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"   # circle_pilot 包根
BF_SRC="${PKG_ROOT}/adapters/bf"
BF_BUILD="${BF_SRC}/build"
BUILD_SH="${PKG_ROOT}/scripts/bf_build.sh"

FLIGHT_TEMPLATE="${SCRIPT_DIR}/bf_flight.service.in"
DEBUG_TEMPLATE="${SCRIPT_DIR}/bf_debugd.service.in"
FLIGHT_UNIT="bf_flight.service"
DEBUG_UNIT="bf_debugd.service"

# ---- 默认（用户级）----
SCOPE="user"
PREFIX=""
JOBS=4
SRC_FLIGHT_CONFIG="${PKG_ROOT}/config/strike_bf_flight.yaml"
SRC_DEBUG_CONFIG="${PKG_ROOT}/config/strike_bf_debug.yaml"
PURGE=0

usage() { sed -n '2,40p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; }

die() { echo "错误: $*" >&2; exit 1; }

# ---- 参数解析 ----
CMD=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    build|package|install|enable|start|stop|disable|uninstall|status|all)
      CMD="$1"; shift ;;
    --user)   SCOPE="user"; shift ;;
    --system) SCOPE="system"; shift ;;
    --prefix) PREFIX="${2:?}"; shift 2 ;;
    --jobs|-j) JOBS="${2:?}"; shift 2 ;;
    --flight-config) SRC_FLIGHT_CONFIG="${2:?}"; shift 2 ;;
    --debug-config)  SRC_DEBUG_CONFIG="${2:?}"; shift 2 ;;
    --purge) PURGE=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) die "未知参数: $1" ;;
  esac
done
[[ -n "${CMD}" ]] || { usage; exit 0; }

# ---- 作用域相关变量 ----
if [[ "${SCOPE}" == "system" ]]; then
  PREFIX="${PREFIX:-/opt/circle}"
  UNIT_DIR="/etc/systemd/system"
  SYSTEMCTL=(systemctl)
  WANTED_BY="multi-user.target"
  PRIORITY_DIRECTIVES=$'Nice=-10\nIOSchedulingClass=realtime\nIOSchedulingPriority=0'
else
  PREFIX="${PREFIX:-${HOME}/.local/share/circle_bf}"
  UNIT_DIR="${HOME}/.config/systemd/user"
  SYSTEMCTL=(systemctl --user)
  WANTED_BY="default.target"
  # 用户级 systemd 无权设置 realtime IO / 负 nice，留空避免单元启动失败。
  PRIORITY_DIRECTIVES=""
fi

need_root_if_system() {
  if [[ "${SCOPE}" == "system" && "${EUID}" -ne 0 ]]; then
    die "系统级操作需 root：请用 sudo $0 ${CMD} --system"
  fi
}

# ---- 从 YAML（2 空格缩进的顶层 key）取值，去注释/引号 ----
yaml_value() {  # <file> <key>
  grep -E "^[[:space:]]+$2:" "$1" 2>/dev/null \
    | grep -vE '^[[:space:]]*#' | head -1 \
    | sed -E "s/^[[:space:]]+$2:[[:space:]]*//" \
    | sed -E 's/[[:space:]]*#.*$//' \
    | sed -E 's/^["'\'']//; s/["'\'']$//' \
    | sed -E 's/[[:space:]]*$//'
}

# 复制一个被配置引用的资源到 PREFIX，保持其相对布局。
# 相对路径 → 相对 PKG_ROOT 解析；绝对路径若位于 PKG_ROOT 下则按相对镜像。
stage_referenced() {  # <value>
  local val="$1" src rel
  [[ -z "${val}" ]] && return 0
  if [[ "${val}" = /* ]]; then
    if [[ "${val}" == "${PKG_ROOT}/"* ]]; then
      rel="${val#"${PKG_ROOT}/"}"
      src="${val}"
    else
      echo "  跳过（绝对且不在包内，请确保目标机存在）: ${val}"
      return 0
    fi
  else
    rel="${val}"
    src="${PKG_ROOT}/${val}"
  fi
  [[ -e "${src}" ]] || { echo "  警告: 引用资源不存在，跳过: ${src}"; return 0; }
  mkdir -p "${PREFIX}/$(dirname "${rel}")"
  cp -af "${src}" "${PREFIX}/${rel}"
  echo "  + ${rel}"
}

# ============ build ============
do_build() {
  [[ -x "${BUILD_SH}" ]] || die "未找到构建脚本: ${BUILD_SH}"
  echo ">>> 编译 BF (${BUILD_SH} -j ${JOBS})"
  "${BUILD_SH}" -j "${JOBS}"
}

# ============ package ============
do_package() {
  [[ -x "${BF_BUILD}/bf_flight" ]] || die "未找到 bf_flight，请先 ./bf_deploy.sh build"
  [[ -x "${BF_BUILD}/bf_debugd" ]] || die "未找到 bf_debugd，请先 ./bf_deploy.sh build"
  [[ -f "${SRC_FLIGHT_CONFIG}" ]] || die "缺少 flight 配置: ${SRC_FLIGHT_CONFIG}"
  [[ -f "${SRC_DEBUG_CONFIG}" ]]  || die "缺少 debug 配置: ${SRC_DEBUG_CONFIG}"

  echo ">>> 打包到 ${PREFIX}"
  mkdir -p "${PREFIX}/bin" "${PREFIX}/config"

  echo "  二进制:"
  install -m 0755 "${BF_BUILD}/bf_flight" "${PREFIX}/bin/bf_flight"
  install -m 0755 "${BF_BUILD}/bf_debugd" "${PREFIX}/bin/bf_debugd"
  echo "  + bin/bf_flight  + bin/bf_debugd"

  echo "  配置:"
  install -m 0644 "${SRC_FLIGHT_CONFIG}" "${PREFIX}/config/strike_bf_flight.yaml"
  install -m 0644 "${SRC_DEBUG_CONFIG}"  "${PREFIX}/config/strike_bf_debug.yaml"
  echo "  + config/strike_bf_flight.yaml  + config/strike_bf_debug.yaml"

  local pkg_flight="${PREFIX}/config/strike_bf_flight.yaml"
  local pkg_debug="${PREFIX}/config/strike_bf_debug.yaml"

  echo "  引用资源:"
  stage_referenced "$(yaml_value "${pkg_flight}" camera)"
  stage_referenced "$(yaml_value "${pkg_flight}" camera_info)"
  stage_referenced "$(yaml_value "${pkg_debug}"  plots_html)"

  # ---- 模型：复制整目录到 PREFIX 并把 model_path 改成相对路径 ----
  local model_val model_rel model_dir
  model_val="$(yaml_value "${pkg_flight}" model_path)"
  if [[ -n "${model_val}" ]]; then
    if [[ "${model_val}" == "${PKG_ROOT}/"* ]]; then
      model_rel="${model_val#"${PKG_ROOT}/"}"
    else
      model_rel="circle_pilot_object_tracking/models/$(basename "$(dirname "${model_val}")")/$(basename "${model_val}")"
    fi
    model_dir="$(dirname "${model_val}")"
    if [[ -d "${model_dir}" ]]; then
      mkdir -p "${PREFIX}/$(dirname "${model_rel}")"
      cp -af "${model_dir}/." "${PREFIX}/$(dirname "${model_rel}")/"
      # 改写打包配置内未注释的 model_path 行为相对路径
      sed -i -E "s|^([[:space:]]+)model_path:[[:space:]]*.*$|\1model_path: ${model_rel}|" "${pkg_flight}"
      echo "  + ${model_rel} (model_path 已改写为相对)"
    else
      echo "  警告: 模型目录不存在，跳过: ${model_dir}"
    fi
  fi

  echo ">>> 打包完成: ${PREFIX}"
}

# ============ systemd 单元渲染/安装 ============
render_unit() {  # <template> <out>
  local prio_block="${PRIORITY_DIRECTIVES}"
  # 占位符在独立一行：有内容则补换行，无内容则整行删除
  if [[ -n "${prio_block}" ]]; then
    awk -v prio="${prio_block}" '
      /@@PRIORITY_DIRECTIVES@@/ { print prio; next }
      { print }
    ' "$1" \
    | sed -e "s|@@PREFIX@@|${PREFIX}|g" -e "s|@@WANTED_BY@@|${WANTED_BY}|g" >"$2"
  else
    grep -v '@@PRIORITY_DIRECTIVES@@' "$1" \
    | sed -e "s|@@PREFIX@@|${PREFIX}|g" -e "s|@@WANTED_BY@@|${WANTED_BY}|g" >"$2"
  fi
}

do_install_units() {
  need_root_if_system
  [[ -f "${FLIGHT_TEMPLATE}" ]] || die "缺少模板: ${FLIGHT_TEMPLATE}"
  [[ -f "${DEBUG_TEMPLATE}" ]]  || die "缺少模板: ${DEBUG_TEMPLATE}"
  [[ -x "${PREFIX}/bin/bf_flight" ]] || die "PREFIX 未打包，请先 ./bf_deploy.sh package"

  mkdir -p "${UNIT_DIR}"
  echo ">>> 渲染 systemd 单元到 ${UNIT_DIR} (${SCOPE})"
  render_unit "${FLIGHT_TEMPLATE}" "${UNIT_DIR}/${FLIGHT_UNIT}"
  render_unit "${DEBUG_TEMPLATE}"  "${UNIT_DIR}/${DEBUG_UNIT}"
  chmod 644 "${UNIT_DIR}/${FLIGHT_UNIT}" "${UNIT_DIR}/${DEBUG_UNIT}"
  "${SYSTEMCTL[@]}" daemon-reload
  echo "  + ${FLIGHT_UNIT}  + ${DEBUG_UNIT}"
}

do_enable() {
  do_install_units
  "${SYSTEMCTL[@]}" enable "${FLIGHT_UNIT}" "${DEBUG_UNIT}"
  echo ">>> 已 enable（开机自启）"
  if [[ "${SCOPE}" == "user" ]]; then
    echo "    提示：用户级服务开机自启需开启 linger:  sudo loginctl enable-linger ${USER}"
  fi
}

do_start() {
  do_enable
  "${SYSTEMCTL[@]}" restart "${FLIGHT_UNIT}" "${DEBUG_UNIT}"
  echo ">>> 已启动 ${FLIGHT_UNIT} + ${DEBUG_UNIT}"
}

do_stop() {
  "${SYSTEMCTL[@]}" stop "${DEBUG_UNIT}" "${FLIGHT_UNIT}" 2>/dev/null || true
  echo ">>> 已停止"
}

do_disable() {
  "${SYSTEMCTL[@]}" disable "${DEBUG_UNIT}" "${FLIGHT_UNIT}" 2>/dev/null || true
  echo ">>> 已禁用"
}

do_uninstall() {
  need_root_if_system
  "${SYSTEMCTL[@]}" stop "${DEBUG_UNIT}" "${FLIGHT_UNIT}" 2>/dev/null || true
  "${SYSTEMCTL[@]}" disable "${DEBUG_UNIT}" "${FLIGHT_UNIT}" 2>/dev/null || true
  rm -f "${UNIT_DIR}/${FLIGHT_UNIT}" "${UNIT_DIR}/${DEBUG_UNIT}"
  "${SYSTEMCTL[@]}" daemon-reload 2>/dev/null || true
  echo ">>> 已卸载单元"
  if [[ "${PURGE}" -eq 1 ]]; then
    rm -rf "${PREFIX}"
    echo ">>> 已删除 PREFIX: ${PREFIX}"
  fi
}

do_status() {
  "${SYSTEMCTL[@]}" status "${FLIGHT_UNIT}" "${DEBUG_UNIT}" --no-pager 2>&1 || true
}

print_summary() {
  echo ""
  echo "作用域:   ${SCOPE}"
  echo "PREFIX:   ${PREFIX}"
  echo "单元目录: ${UNIT_DIR}"
  echo "systemctl: ${SYSTEMCTL[*]}"
}

case "${CMD}" in
  build)     do_build ;;
  package)   do_package; print_summary ;;
  install)   do_package; do_install_units; print_summary ;;
  enable)    do_package; do_enable; print_summary ;;
  start)     do_package; do_start; print_summary ;;
  stop)      do_stop ;;
  disable)   do_disable ;;
  uninstall) do_uninstall ;;
  status)    do_status ;;
  all)       do_build; do_package; do_start; print_summary
             jflag="--user"; [[ "${SCOPE}" == "system" ]] && jflag="--system"
             echo ""
             echo ">>> 查看日志:"
             echo "    ${SYSTEMCTL[*]} status ${FLIGHT_UNIT}"
             echo "    journalctl ${jflag} -u ${FLIGHT_UNIT} -f" ;;
esac
