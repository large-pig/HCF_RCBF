/**
 * @file torque_limits.h
 * @brief 关节力矩幅值与力矩变化率约束。
 *
 * 约束区间的单位为 N·m，变化率计算使用上一周期下发的力矩。
 * 控制器先换算允许的加速度范围，最终发送力矩时再次执行幅值和变化率约束。
 */

#pragma once

#include "core.h"

namespace tase_hcf_rcbf {
struct TorqueInterval {
  double lower = 0;
  double upper = 0;
  bool valid = false;
};
// 取力矩幅值限制与单周期变化率限制的交集。
inline TorqueInterval torqueInterval(double previous, double amplitude, double rate, double dt) {
  if (!finite(previous) || !finite(amplitude) || amplitude <= 0 || !finite(rate) || rate <= 0 ||
      !finite(dt) || dt <= 0)
    return {};
  TorqueInterval x;
  x.lower = std::max(-amplitude, previous - rate * dt);
  x.upper = std::min(amplitude, previous + rate * dt);
  x.valid = x.lower <= x.upper;
  return x;
}
// 投影请求力矩；若区间不一致，采用符合变化率限制的恢复策略。
inline double boundedHold(double target, double previous, double amplitude, double rate,
                          double dt) {
  const auto b = torqueInterval(previous, amplitude, rate, dt);
  if (b.valid)
    return clamp(target, b.lower, b.upper);
  if (!finite(previous))
    return 0;
  // 若上一周期力矩位于新幅值限制之外，
  // 则按照变化率上限逐步靠近允许的目标力矩。
  const double step = finite(dt) && dt > 0 ? rate * dt : 0;
  return clamp(clamp(target, -amplitude, amplitude), previous - step, previous + step);
}
}  // 命名空间 tase_hcf_rcbf
