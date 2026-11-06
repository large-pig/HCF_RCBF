#!/usr/bin/env bash
set -euo pipefail
if [[ ( $# != 3 && $# != 4 ) || "$1" != '--hardware' ]]; then
  echo 'Usage: bash run_moving_hcf_p2_7.sh --hardware PERIODIC|BURST|BIAS_PERIODIC ROBOT_IP [NEW_TRIAL_ID]' >&2
  exit 2
fi
case "$2" in PERIODIC|BURST|BIAS_PERIODIC) ;; *) exit 2 ;; esac
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ "$(realpath "$(rospack find tase_hcf_rcbf)")" != "$(realpath "$root/tase_hcf_rcbf")" ]]; then
  echo 'Source this package’s .build/ros_ws/devel/setup.bash first.' >&2
  exit 2
fi
folder="$HOME/tase_moving_data"
trial="${4:-TV_${2}_HCF_SIN05_P1_7_P2_7_KE3_E15_E150_R01}"
[[ "$trial" =~ ^[A-Za-z0-9_-]+$ && "$trial" != *unset* ]] || exit 2
mkdir -p "$folder"
for path in "$folder/$trial.bag" "$folder/$trial.bag.active" "$folder/${trial}_parameters.json"; do
  if [[ -e "$path" ]]; then
    echo 'Existing trial ID. Preserve the previous recording.' >&2
    exit 2
  fi
done
# HCF 不使用 RCBF 的加速度扰动预算，保持该模式约定的基础配置。
# 扰动波形、轨迹、执行器限制和运行监控均由最终 launch 统一设置。
exec roslaunch tase_hcf_rcbf disturbance_comparison.launch enable_hardware:=true \
  robot_ip:="$3" mode:=HCF waveform:="$2" base_budget:=0.0 \
  tuning_file:="$root/tase_hcf_rcbf/config/hcf_ke3_eta15_eta150_p2_7.yaml" \
  trial_id:="$trial" bag_prefix:="$folder/$trial"
