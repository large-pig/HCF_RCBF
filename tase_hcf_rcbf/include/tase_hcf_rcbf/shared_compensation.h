/**
 * @file shared_compensation.h
 * @brief HCF 与 RCBF 共用的有界前馈补偿。
 *
 * 根据标定的速度相关模型计算补偿力矩，并实施启动渐入、幅值限制和变化率限制。
 * 该模块不负责估计外部扰动，两个控制器使用相同的补偿计算。
 */

#pragma once

#include "signals.h"

namespace tase_hcf_rcbf {
// HCF 与 RCBF 共用同一套已标定的前馈补偿参数，不使用扰动观测器输出。
struct SharedCompensation {
  bool enabled = false;
  double offset = .1256390057;
  double coulomb = 1.1509920922;
  double viscous = 0;
  double velocity_scale = .003;
  double scale = .1;
  double ramp_s = 5;
  double cap = .15;
  double rate = 1;
  double value = 0;
  // 检查标定参数的有效范围，避免使用超出标定范围的补偿量。
  bool valid() const {
    for (double x : {offset, coulomb, viscous, velocity_scale, scale, ramp_s, cap, rate})
      if (!finite(x))
        return false;
    return coulomb >= 0 && viscous >= 0 && velocity_scale > 0 && scale >= 0 && scale <= .1 &&
           ramp_s >= 5 && cap > 0 && cap <= .15 && rate > 0 && rate <= 1;
  }
  // 依次执行渐入、力矩幅值限制和变化率限制；停止运动后平滑回零。
  double update(double velocity, double t, double dt, bool running) {
    if (!finite(velocity) || !finite(t) || !finite(dt) || dt <= 0)
      return value;
    double target =
        enabled && running
            ? scale * ramp(t, ramp_s).value *
                  (offset + coulomb * std::tanh(velocity / velocity_scale) + viscous * velocity)
            : 0;
    target = clamp(target, -cap, cap);
    value = clamp(target, value - rate * dt, value + rate * dt);
    return value;
  }
};
}  // 命名空间 tase_hcf_rcbf
