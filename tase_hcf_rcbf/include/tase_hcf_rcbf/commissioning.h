/**
 * @file commissioning.h
 * @brief 实机启动检查、运动范围监控和参考轨迹平滑处理。
 *
 * 启动与运动检查独立于 CBF 控制算法运行，用于实验过程中的状态监控。
 * 关节运动约束不等同于整机工作空间的碰撞安全保证。
 */

#pragma once

#include <array>

#include "signals.h"

namespace tase_hcf_rcbf {
// 使用二阶连续的平滑包络，轨迹结束时返回设定的中心位置。
// 从 0 平滑过渡至 1，避免参考轨迹在切换时产生尖锐变化。
inline Sample quinticRise(double t, double T) {
  if (t <= 0)
    return {};
  if (t >= T)
    return {1, 0, 0};
  const double s = t / T;
  return {s * s * s * (10 + s * (-15 + 6 * s)), 30 * s * s * (1 - s) * (1 - s) / T,
          60 * s * (1 - s) * (1 - 2 * s) / (T * T)};
}
inline Sample commissioningReference(const Harmonic& h, double t, double duration, double ramp_s) {
  if (ramp_s <= 0)
    return sample(h, t);
  if (t <= 0 || t >= duration)
    return {h.offset, 0, 0};
  auto e = quinticRise(t, ramp_s);
  if (t > duration - ramp_s) {
    e = quinticRise(duration - t, ramp_s);
    e.derivative = -e.derivative;
  }
  const auto a = sample(h, t);
  const double x = a.value - h.offset;
  return {h.offset + e.value * x, e.derivative * x + e.value * a.derivative,
          e.second * x + 2 * e.derivative * a.derivative + e.value * a.second};
}
struct CommissioningGuard {
  bool enabled = false;
  // 只有执行轨迹的受控关节允许按已验证配置关闭部分运动检查。
  // 初始状态、其他关节和保持阶段的检查仍保持启用。
  bool controlled_joint_excursion_enabled = true;
  bool controlled_joint_speed_enabled = true;
  double initial_position_tolerance = .02;
  double initial_speed = .02;
  double excursion = .2;
  double max_speed = .2;
  double other_joint_drift = .04;
};
inline bool validGuard(const CommissioningGuard& g) {
  const double a[] = {g.initial_position_tolerance, g.initial_speed, g.excursion, g.max_speed,
                      g.other_joint_drift};
  for (double x : a)
    if (!finite(x) || x <= 0)
      return false;
  return g.initial_position_tolerance < g.excursion && g.initial_speed <= g.max_speed;
}
// 检查所有关节的初始速度与受控关节的初始位置。
inline bool commissioningInitialOk(const CommissioningGuard& g, const std::array<double, 7>& q,
                                   const std::array<double, 7>& dq, int joint, double centre,
                                   bool holding) {
  if (!g.enabled)
    return true;
  for (int j = 0; j < 7; ++j)
    if (!finite(q[j]) || !finite(dq[j]) || std::abs(dq[j]) > g.initial_speed)
      return false;
  return holding || std::abs(q[joint] - centre) <= g.initial_position_tolerance;
}
// 运行过程中持续监测关节偏移、速度以及其他关节漂移。
inline bool commissioningMotionOk(const CommissioningGuard& g, const std::array<double, 7>& q,
                                  const std::array<double, 7>& dq,
                                  const std::array<double, 7>& start, int joint, double centre,
                                  bool holding) {
  if (!g.enabled)
    return true;
  for (int j = 0; j < 7; ++j) {
    if (!finite(q[j]) || !finite(dq[j]))
      return false;
    const bool moving_joint = j == joint && !holding;
    if ((!moving_joint || g.controlled_joint_speed_enabled) && std::abs(dq[j]) > g.max_speed)
      return false;
    if (moving_joint && !g.controlled_joint_excursion_enabled)
      continue;
    const double target = moving_joint ? centre : start[j];
    const double bound = moving_joint ? g.excursion : g.other_joint_drift;
    if (std::abs(q[j] - target) > bound)
      return false;
  }
  return true;
}
}  // 命名空间 tase_hcf_rcbf
