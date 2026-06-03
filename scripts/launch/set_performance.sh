#!/usr/bin/env bash
# RK3588 NPU / DDR / CPU 频率锁定为 performance 模式。
# 消除 DVFS 动态调频导致的推理延迟尖峰（infer 47ms → ~24ms 稳定）。
# 需要 sudo 权限；多次调用幂等（不会报错）。
# 用法：直接执行，或由 launch_common.sh 自动调用。

_set_governor() {
  local path="$1"
  local name="$2"
  if [[ -f "${path}/governor" ]]; then
    local cur
    cur=$(cat "${path}/governor" 2>/dev/null)
    if [[ "${cur}" != "performance" ]]; then
      if echo performance | sudo tee "${path}/governor" > /dev/null 2>&1; then
        local freq
        freq=$(cat "${path}/cur_freq" 2>/dev/null)
        echo "  ${name}: ${cur} → performance (${freq} Hz)"
      else
        echo "  ${name}: 设置失败（需要 sudo 权限）" >&2
        return 1
      fi
    else
      local freq
      freq=$(cat "${path}/cur_freq" 2>/dev/null)
      echo "  ${name}: 已是 performance (${freq} Hz)"
    fi
  else
    echo "  ${name}: 路径不存在 (${path})" >&2
  fi
}

echo "[perf] 锁定 RK3588 频率为 performance 模式..."

_set_governor "/sys/class/devfreq/fdab0000.npu" "NPU"
_set_governor "/sys/class/devfreq/dmc" "DDR"

for policy_dir in /sys/devices/system/cpu/cpufreq/policy*; do
  if [[ -d "${policy_dir}" ]]; then
    policy_name=$(basename "${policy_dir}")
    _set_governor "${policy_dir}" "CPU/${policy_name}"
  fi
done

# 温度提示
for tz in /sys/class/thermal/thermal_zone*/temp; do
  if [[ -f "${tz}" ]]; then
    zone=$(echo "${tz}" | grep -oP 'thermal_zone\d+')
    temp=$(cat "${tz}" 2>/dev/null)
    if [[ -n "${temp}" && "${temp}" -gt 0 ]]; then
      echo "  温度 ${zone}: $((temp / 1000))°C"
    fi
  fi
done

echo "[perf] 完成。注意：performance 模式功耗增加，请确保散热充足。"
