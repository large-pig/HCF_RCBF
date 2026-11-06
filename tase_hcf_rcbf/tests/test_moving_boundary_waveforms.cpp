/**
 * @file test_moving_boundary_waveforms.cpp
 * @brief 最终扰动波形下的动态边界离线检查。
 *
 * 本文件仅验证控制逻辑，不代替实机实验，也不构成整机碰撞安全认证。
 */

#include <tase_hcf_rcbf/controlled_injection.h>
#include <tase_hcf_rcbf/torque_limits.h>

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace tase_hcf_rcbf;

namespace {

void check(bool condition, const char* message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

Config finalConfig(Mode mode) {
  Config config;
  config.mode = mode;
  config.kp = 1600.0;
  config.kd = 320.0;

  if (mode == Mode::HCF) {
    config.ke = 3.0;
    config.eta1 = 15.0;
    config.eta2 = 150.0;
    config.p1 = 7.0;
    config.p2 = 7.0;
    config.physical_accel_bound = 2.0;
  } else {
    config.p1 = 7.0;
    config.p2 = 10.0;
    config.k1 = 50.0;
    config.k2 = 50.0;
    config.physical_accel_bound = 30.0;
  }

  return config;
}

}  // namespace

int main() {
  const Harmonic reference{0.0, 1.0, 0.5, false};
  const Harmonic lower{-0.5, 0.4, 0.5, false};
  const Harmonic upper{0.6, 0.1, 1.0, true};

  constexpr double duration = 50.0;
  constexpr double dt = 0.001;
  constexpr double equivalent_inertia = 0.023;

  // 参考轨迹经五秒平滑包络后启动和结束，端点速度为零。
  for (double time : {0.0, duration}) {
    const auto sample = commissioningReference(reference, time, duration, 5.0);
    check(sample.value == 0.0 && sample.derivative == 0.0 && sample.second == 0.0,
          "reference endpoints must be stationary");
  }

  Disturbance injection;
  injection.enabled = true;
  injection.profile = "sine";
  injection.units = "acceleration";
  injection.amplitude = 2.0;
  injection.omega = 0.5;
  injection.ramp_s = 5.0;
  check(validControlledInjection(injection, duration), "final D1 injection configuration");

  // 只覆盖最终三种扰动波形及两种控制器配置。
  for (const std::string scenario : {"PERIODIC", "BURST", "BIAS_PERIODIC"}) {
    const InjectionWaveform waveform = parseWaveform(scenario);
    check(waveform != InjectionWaveform::Invalid, "known disturbance waveform");
    check(shapedInjectionValue(waveform, injection, 0.0, duration) == 0.0,
          "injection starts at zero");
    check(shapedInjectionValue(waveform, injection, duration, duration) == 0.0,
          "injection ends at zero");

    for (Mode mode : {Mode::HCF, Mode::RCBF}) {
      Config config = finalConfig(mode);
      State state;

      double position = 0.0;
      double velocity = 0.0;
      double previous_acceleration = 0.0;
      double previous_torque = 0.0;
      double minimum_margin = 1.0;
      double peak_injection = 0.0;

      for (int cycle = 0; cycle < static_cast<int>(duration / dt); ++cycle) {
        const double time = cycle * dt;
        const auto desired = commissioningReference(reference, time, duration, 5.0);
        const auto lower_sample = sample(lower, time);
        const auto upper_sample = sample(upper, time);
        const double disturbance = shapedInjectionValue(waveform, injection, time, duration);

        check(std::abs(disturbance) <= 2.0 + 1e-12,
              "injected acceleration exceeds its configured amplitude");
        if (waveform == InjectionWaveform::Burst && (time <= 26.0 || time >= 30.0)) {
          check(disturbance == 0.0, "burst injection is zero outside its support");
        }
        peak_injection = std::max(peak_injection, std::abs(disturbance));

        const TorqueInterval torque = torqueInterval(previous_torque, 12.0, 999.999, dt);
        check(torque.valid, "valid joint-5 torque interval");

        Input input;
        input.q = position;
        input.velocity = velocity;
        input.qd = desired.value;
        input.dqd = desired.derivative;
        input.ddqd = desired.second;
        input.lower = lower_sample.value;
        input.dlower = lower_sample.derivative;
        input.ddlower = lower_sample.second;
        input.upper = upper_sample.value;
        input.dupper = upper_sample.derivative;
        input.ddupper = upper_sample.second;
        input.dt = dt;
        input.previous_nominal_accel = previous_acceleration;
        input.accel_min = torque.lower / equivalent_inertia - disturbance;
        input.accel_max = torque.upper / equivalent_inertia - disturbance;

        const Output output = step(config, input, state);
        check(output.valid, "final controller setting became infeasible in the ideal-model test");
        if (cycle == 0) {
          check(output.initial_safe, "initial position must satisfy both moving boundaries");
        }

        minimum_margin = std::min(
            minimum_margin,
            std::min(position - lower_sample.value, upper_sample.value - position));
        previous_acceleration = output.command_accel;
        previous_torque = equivalent_inertia * (previous_acceleration + disturbance);

        const double acceleration = previous_acceleration + disturbance;
        position += velocity * dt + 0.5 * acceleration * dt * dt;
        velocity += acceleration * dt;
      }

      check(peak_injection > 1.0, "disturbance waveform was not activated");
      check(minimum_margin >= -1e-4,
            "ideal-model trajectory violated a moving boundary beyond numerical tolerance");
    }
  }

  std::cout << "PASS: final HCF/RCBF settings across PERIODIC, BURST and BIAS_PERIODIC\n";
  return 0;
}
