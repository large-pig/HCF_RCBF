#!/usr/bin/env bash
# 构建并执行离线控制器测试；不会连接或启动真实机器人。
set -euo pipefail

if [[ $# -gt 1 || ( $# -eq 1 && "$1" != "--ros" ) ]]; then
  echo "Usage: bash run_offline.sh [--ros]" >&2
  exit 2
fi

root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
pkg="$root/tase_hcf_rcbf"
build="$root/.build"
mkdir -p "$build"
touch "$build/CATKIN_IGNORE"

# 核心测试无需 ROS Master 或机器人硬件，显式列出测试目标，
# 避免未来添加的模块未经检查就进入构建流程。
for test in \
  test_core \
  test_commissioning \
  test_shared_compensation \
  test_controlled_injection \
  test_moving_boundary_waveforms; do
  g++ -std=c++14 -O2 -Wall -Wextra -Werror \
    -I "$pkg/include" "$pkg/tests/${test}.cpp" -o "$build/$test"
  "$build/$test"
done

python3 "$pkg/scripts/analyze_trials.py" self-test
python3 "$pkg/tests/test_final_configuration.py"
python3 -m py_compile \
  "$root/start_trial.py" \
  "$root/collect_return.py" \
  "$pkg/scripts/analyze_trials.py" \
  "$pkg/tests/test_final_configuration.py"

if [[ "${1:-}" == "--ros" ]]; then
  set +u
  source /opt/ros/noetic/setup.bash
  set -u

  export CMAKE_PREFIX_PATH=/opt/ros/noetic
  export ROS_PACKAGE_PATH=/opt/ros/noetic/share
  export PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages
  export LD_LIBRARY_PATH=/opt/ros/noetic/lib:/opt/ros/noetic/lib/x86_64-linux-gnu
  export ROS_HOME="$build/ros"
  export ROS_LOG_DIR="$ROS_HOME/log"

  mkdir -p "$build/ros_ws/src"
  if [[ ! -e "$build/ros_ws/src/tase_hcf_rcbf" ]]; then
    ln -s "$pkg" "$build/ros_ws/src/tase_hcf_rcbf"
  fi

  catkin_make -C "$build/ros_ws" --pkg tase_hcf_rcbf \
    -DCATKIN_ENABLE_TESTING=ON -j2 -l2
  (cd "$build/ros_ws/build/tase_hcf_rcbf" && ctest --output-on-failure)
fi

echo "PASS: offline checks completed; no robot was connected."
