#!/usr/bin/env bash
set -euo pipefail
if [[ ( $# != 3 && $# != 4 ) || "$1" != '--hardware' ]]; then
  echo 'Usage: bash run_moving_rcbf_b30_k50.sh --hardware PERIODIC|BURST|BIAS_PERIODIC ROBOT_IP [NEW_TRIAL_ID]' >&2
  exit 2
fi
case "$2" in PERIODIC|BURST|BIAS_PERIODIC) ;; *) exit 2 ;; esac
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ "$(realpath "$(rospack find tase_hcf_rcbf)")" != "$(realpath "$root/tase_hcf_rcbf")" ]]; then
  echo 'Source this package’s .build/ros_ws/devel/setup.bash first.' >&2
  exit 2
fi
folder="$HOME/tase_moving_data"
trial="${4:-TV_${2}_RCBF_SIN05_B30_K50_R01}"
[[ "$trial" =~ ^[A-Za-z0-9_-]+$ && "$trial" != *unset* ]] || exit 2
mkdir -p "$folder"
for path in "$folder/$trial.bag" "$folder/$trial.bag.active" "$folder/${trial}_parameters.json"; do
  if [[ -e "$path" ]]; then
    echo 'Existing trial ID. Preserve the previous recording.' >&2
    exit 2
  fi
done
# RCBF 预算取剩余界 28 加软件注入界 2，总计 B=30 rad/s²。
# 这是实验采用的预算假设，不代表全部物理残余扰动已经辨识。
exec roslaunch tase_hcf_rcbf disturbance_comparison.launch enable_hardware:=true \
  robot_ip:="$3" mode:=RCBF waveform:="$2" base_budget:=28.0 \
  tuning_file:="$root/tase_hcf_rcbf/config/rcbf_b30_k50.yaml" \
  trial_id:="$trial" bag_prefix:="$folder/$trial"
