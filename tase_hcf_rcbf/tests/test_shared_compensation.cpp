/**
 * @file test_shared_compensation.cpp
 * @brief 公共前馈补偿的幅值、变化率、启停及参数校验测试。
 *
 * 本测试只验证独立补偿模块，不包含真实机器人或闭环动力学仿真。
 */

#include <tase_hcf_rcbf/shared_compensation.h>

#include <cmath>
#include <stdexcept>

namespace {

void require(bool condition, const char* description) {
  if (!condition) {
    throw std::runtime_error(description);
  }
}

}  // namespace

int main() {
  using tase_hcf_rcbf::SharedCompensation;

  SharedCompensation compensation;
  require(compensation.valid(), "Default compensation parameters must be valid");

  // 禁用补偿后，不得向力矩命令中添加前馈项。
  require(compensation.update(0.05, 10.0, 0.001, true) == 0.0,
          "Disabled compensation must remain zero");

  compensation.enabled = true;
  constexpr double dt = 0.001;

  // 检查两个速度方向及五秒渐入阶段的补偿响应。
  // 验证力矩幅值上限和逐周期变化率限制。
  for (int sample = 0; sample < 10000; ++sample) {
    const double time = sample * dt;
    const double velocity = sample < 5000 ? 0.05 : -0.05;
    const double previous = compensation.value;
    const double current = compensation.update(velocity, time, dt, true);

    require(std::abs(current) <= compensation.cap + 1e-12,
            "Compensation torque exceeded its amplitude cap");
    require(std::abs(current - previous) <= compensation.rate * dt + 1e-12,
            "Compensation torque exceeded its slew-rate limit");
  }

  // 结束运动后补偿值应平滑回到零。
  for (int sample = 0; sample < 1000; ++sample) {
    compensation.update(0.0, 10.0 + sample * dt, dt, false);
  }
  require(std::abs(compensation.value) < 1e-12,
          "Compensation did not return to zero after stopping");

  // 拒绝超出已标定参数区间的配置。
  compensation.scale = 0.2;
  require(!compensation.valid(), "Over-limit compensation scale was accepted");

  return 0;
}
