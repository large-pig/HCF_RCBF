/**
 * @file signals.h
 * @brief 时变参考轨迹、动态安全边界与扰动信号。
 *
 * 同时计算信号值及其一、二阶导数，保证 HCF 和 RCBF 使用一致的边界运动学量。
 */

#pragma once

#include <string>

#include "core.h"

namespace tase_hcf_rcbf {
constexpr double PI = 3.14159265358979323846;
struct Harmonic {
  double offset = 0;
  double amplitude = 0;
  double omega = 0;
  bool cosine = false;
};
struct Sample {
  double value = 0;
  double derivative = 0;
  double second = 0;
};
inline bool validHarmonic(const Harmonic& h) {
  return finite(h.offset) && finite(h.amplitude) && finite(h.omega) && h.omega >= 0;
}
// 计算谐波信号及其一阶、二阶时间导数。
inline Sample sample(const Harmonic& h, double t) {
  const double s = std::sin(h.omega * t), c = std::cos(h.omega * t), v = h.cosine ? c : s;
  return {h.offset + h.amplitude * v, h.amplitude * h.omega * (h.cosine ? -s : c),
          -h.amplitude * h.omega * h.omega * v};
}
inline Sample ramp(double t, double duration) {
  if (t >= duration)
    return {1, 0, 0};
  if (t <= 0)
    return {0, 0, 0};
  return {.5 * (1 - std::cos(PI * t / duration)), .5 * PI / duration * std::sin(PI * t / duration),
          .5 * PI * PI / (duration * duration) * std::cos(PI * t / duration)};
}
struct Disturbance {
  bool enabled = false;
  std::string profile = "case1", units = "acceleration";
  double amplitude = 0;
  double omega = 1;
  double scale = 1;
  double ramp_s = 2;
};
inline bool validDisturbance(const Disturbance& d) {
  return (d.profile == "case1" || d.profile == "case2" || d.profile == "sine") &&
         (d.units == "torque" || d.units == "acceleration") && finite(d.amplitude) &&
         finite(d.omega) && d.omega >= 0 && finite(d.scale) && d.scale >= 0 && finite(d.ramp_s) &&
         d.ramp_s > 0;
}
inline double disturbanceBound(const Disturbance& d) {
  return d.enabled
             ? d.scale *
                   (d.profile == "case1" ? 1.1 : (d.profile == "case2" ? 6 : std::abs(d.amplitude)))
             : 0;
}
// 按参数指定的单位生成软件注入扰动波形。
inline double disturbanceValue(const Disturbance& d, double t) {
  if (!d.enabled)
    return 0;
  const double v =
      d.profile == "case1"
          ? std::sin(t) + .1 * std::cos(.5 * t)
          : (d.profile == "case2" ? 2 * std::sin(t) + 2 * std::cos(2 * t) + 2 * std::sin(3 * t)
                                  : d.amplitude * std::sin(d.omega * t));
  return d.scale * ramp(t, d.ramp_s).value * v;
}
struct VelocityBias {
  bool enabled = false;
  double amplitude = .02;
  double omega = 1;
  double ramp_s = 2;
};
inline bool validBias(const VelocityBias& b) {
  return finite(b.amplitude) && finite(b.omega) && b.omega >= 0 && finite(b.ramp_s) && b.ramp_s > 0;
}
inline Sample biasSample(const VelocityBias& b, double t) {
  if (!b.enabled)
    return {};
  const auto r = ramp(t, b.ramp_s);
  const double s = std::sin(b.omega * t), c = std::cos(b.omega * t);
  return {b.amplitude * r.value * s, b.amplitude * (r.derivative * s + r.value * b.omega * c), 0};
}
inline double biasBound(const VelocityBias& b) {
  return b.enabled ? std::abs(b.amplitude) : 0;
}
inline double biasDerivativeBound(const VelocityBias& b) {
  return b.enabled ? std::abs(b.amplitude) * (b.omega + PI / (2 * b.ramp_s)) : 0;
}
struct FrictionModel {
  double coulomb = 0;
  double viscous = 0;
  double smooth_velocity = .02;
};
inline bool validFriction(const FrictionModel& f) {
  return finite(f.coulomb) && f.coulomb >= 0 && finite(f.viscous) && f.viscous >= 0 &&
         finite(f.smooth_velocity) && f.smooth_velocity > 0;
}
// 使用平滑库仑摩擦与黏性摩擦模型计算用户层补偿。
inline double frictionTorque(const FrictionModel& f, double physical_velocity) {
  return f.coulomb * std::tanh(physical_velocity / f.smooth_velocity) +
         f.viscous * physical_velocity;
}
}  // 命名空间 tase_hcf_rcbf
