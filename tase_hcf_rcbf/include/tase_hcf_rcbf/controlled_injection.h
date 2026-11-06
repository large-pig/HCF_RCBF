/**
 * @file controlled_injection.h
 * @brief 实验所用的可控加速度扰动波形。
 *
 * 扰动首先在加速度域生成，单位为 rad/s²。
 * 实时控制器将其乘以关节等效惯量后注入力矩命令。
 */

#pragma once

#include "commissioning.h"

namespace tase_hcf_rcbf {
// 验证实机实验中加速度域注入的波形与参数是否合法。
inline bool validControlledInjection(const Disturbance& d, double duration) {
  return validDisturbance(d) && d.profile == "sine" && d.units == "acceleration" && d.scale == 1 &&
         d.amplitude >= 0 && d.amplitude <= 4 && d.omega >= .2 && d.omega <= 1 && d.ramp_s >= 5 &&
         2 * d.ramp_s < duration;
}
inline double injectionEnvelope(double t, double duration, double ramp_s) {
  return quinticRise(t, ramp_s).value * quinticRise(duration - t, ramp_s).value;
}
inline double controlledInjectionValue(const Disturbance& d, double t, double duration) {
  return d.enabled ? d.amplitude * injectionEnvelope(t, duration, d.ramp_s) * std::sin(d.omega * t)
                   : 0;
}

enum class InjectionWaveform { Periodic, Burst, BiasPeriodic, Invalid };

inline InjectionWaveform parseWaveform(const std::string& name) {
  if (name == "PERIODIC")
    return InjectionWaveform::Periodic;
  if (name == "BURST")
    return InjectionWaveform::Burst;
  if (name == "BIAS_PERIODIC")
    return InjectionWaveform::BiasPeriodic;
  return InjectionWaveform::Invalid;
}

// 最终实验的注入加速度幅值为 2 rad/s²，RCBF 总预算 B30
// 对应三种对比波形；其他预算分支仅保留算法兼容逻辑。
inline bool validMovingResidualBudget(Mode mode, double residual,
                                      InjectionWaveform waveform = InjectionWaveform::Invalid) {
  return ((mode == Mode::HCF || mode == Mode::RCBF) && residual == 0) ||
         (mode == Mode::RCBF &&
          (residual == 1 || (residual == 6 && waveform == InjectionWaveform::Periodic) ||
           (residual == 28 &&
            (waveform == InjectionWaveform::Periodic || waveform == InjectionWaveform::Burst ||
             waveform == InjectionWaveform::BiasPeriodic))));
}

inline double shapedInjectionEnvelope(InjectionWaveform waveform, double t, double duration,
                                      double ramp_s) {
  if (waveform == InjectionWaveform::Burst)
    return quinticRise(t - 26, 2).value * quinticRise(30 - t, 2).value;
  return injectionEnvelope(t, duration, ramp_s);
}

// 根据统一时间包络生成 PERIODIC、BURST 或 BIAS_PERIODIC 扰动。
inline double shapedInjectionValue(InjectionWaveform waveform, const Disturbance& d, double t,
                                   double duration) {
  if (!d.enabled || waveform == InjectionWaveform::Invalid)
    return 0;
  const double envelope = shapedInjectionEnvelope(waveform, t, duration, d.ramp_s);
  if (waveform == InjectionWaveform::Burst)
    return -d.amplitude * envelope;
  const double periodic = std::sin(d.omega * t);
  if (waveform == InjectionWaveform::Periodic)
    return d.amplitude * envelope * periodic;
  const double bias = .6 * std::sin(.08 * t);
  return d.amplitude * envelope * (bias + .4 * periodic);
}
}  // 命名空间 tase_hcf_rcbf
