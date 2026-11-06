/**
 * @file test_controlled_injection.cpp
 * @brief 注入波形幅值与扰动参数合法性测试。
 *
 * 本文件仅验证控制逻辑，不代替实机实验，也不构成整机碰撞安全认证。
 */

#include <tase_hcf_rcbf/controlled_injection.h>

#include <iostream>
#include <stdexcept>
using namespace tase_hcf_rcbf;
void check(bool x, const char* m) {
  if (!x)
    throw std::runtime_error(m);
}
int main() {
  check(validMovingResidualBudget(Mode::HCF, 0) && validMovingResidualBudget(Mode::RCBF, 0),
        "existing B2 budgets retained");
  check(validMovingResidualBudget(Mode::RCBF, 1), "RCBF B3 budget accepted");
  check(validMovingResidualBudget(Mode::RCBF, 6, InjectionWaveform::Periodic),
        "PERIODIC RCBF B8 budget accepted");
  for (auto waveform :
       {InjectionWaveform::Periodic, InjectionWaveform::Burst, InjectionWaveform::BiasPeriodic}) {
    check(validMovingResidualBudget(Mode::RCBF, 28, waveform),
          "named RCBF B30 comparison accepted");
    check(!validMovingResidualBudget(Mode::HCF, 28, waveform), "B30 restricted to RCBF");
  }
  check(!validMovingResidualBudget(Mode::RCBF, 28), "B30 requires a named comparison waveform");
  check(!validMovingResidualBudget(Mode::RCBF, 6, InjectionWaveform::Burst) &&
            !validMovingResidualBudget(Mode::RCBF, 6, InjectionWaveform::BiasPeriodic) &&
            !validMovingResidualBudget(Mode::RCBF, 6) &&
            !validMovingResidualBudget(Mode::HCF, 6, InjectionWaveform::Periodic),
        "B8 restricted to PERIODIC RCBF");
  check(!validMovingResidualBudget(Mode::HCF, 1) && !validMovingResidualBudget(Mode::RCBF, -1) &&
            !validMovingResidualBudget(Mode::RCBF, 2) &&
            !validMovingResidualBudget(Mode::RCBF, std::numeric_limits<double>::quiet_NaN()),
        "reject unauthorized budgets");
  for (bool lower : {true, false}) {
    Config c;
    c.mode = Mode::RCBF;
    c.kp = 1600;
    c.kd = 320;
    c.k1 = c.k2 = 30;
    c.physical_accel_bound = 2;
    Input i;
    i.q = lower ? -.49 : .49;
    i.velocity = lower ? -.05 : .05;
    i.qd = lower ? -1 : 1;
    i.lower = -.5;
    i.upper = .5;
    i.dt = .001;
    i.accel_min = -100;
    i.accel_max = 100;
    State s2, s3;
    auto b2 = step(c, i, s2);
    c.physical_accel_bound = 3;
    auto b3 = step(c, i, s3);
    check(b2.valid && b3.valid, "B2 and B3 projection feasible");
    check(std::abs(b3.barrier_lower - b2.barrier_lower - 1) < 1e-10 &&
              std::abs(b3.barrier_upper - b2.barrier_upper + 1) < 1e-10,
          "B3 tightens both constraints by one");
    check(std::abs(b3.command_accel - b2.command_accel - (lower ? 1 : -1)) < 1e-10,
          "B3 changes active control projection");
    State s8;
    c.physical_accel_bound = 8;
    auto b8 = step(c, i, s8);
    check(b8.valid, "B8 projection feasible");
    check(std::abs(b8.barrier_lower - b2.barrier_lower - 6) < 1e-10 &&
              std::abs(b8.barrier_upper - b2.barrier_upper + 6) < 1e-10,
          "B8 tightens both constraints by six");
    check(std::abs(b8.command_accel - b2.command_accel - (lower ? 6 : -6)) < 1e-10,
          "B8 changes active control projection");
    State s30;
    c.physical_accel_bound = 30;
    auto b30 = step(c, i, s30);
    check(b30.valid, "B30 projection feasible");
    check(std::abs(b30.barrier_lower - b2.barrier_lower - 28) < 1e-10 &&
              std::abs(b30.barrier_upper - b2.barrier_upper + 28) < 1e-10,
          "B30 tightens both constraints by 28");
    check(std::abs(b30.command_accel - b2.command_accel - (lower ? 28 : -28)) < 1e-10,
          "B30 changes active control projection");
  }
  Disturbance d;
  d.enabled = true;
  d.profile = "sine";
  d.amplitude = 4;
  d.omega = .5;
  d.ramp_s = 5;
  check(validControlledInjection(d, 50), "valid D2");
  for (double t : {-1., 0., 50., 51.})
    check(controlledInjectionValue(d, t, 50) == 0, "smooth endpoint zero");
  double peak = 0;
  for (int n = 0; n < 50000; n++) {
    double y = controlledInjectionValue(d, n * .001, 50);
    peak = std::max(peak, std::abs(y));
    check(std::abs(y) <= 4 + 1e-12, "known command bound");
  }
  check(peak > 3.9, "injection actually active");
  d.amplitude = 4.1;
  check(!validControlledInjection(d, 50), "amplitude guard");
  d.amplitude = 2;
  d.omega = 2;
  check(!validControlledInjection(d, 50), "frequency guard");
  std::cout << "PASS: injection amplitude, endpoints, activation and parameter rejection\n";
}
