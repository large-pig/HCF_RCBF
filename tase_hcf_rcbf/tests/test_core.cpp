/**
 * @file test_core.cpp
 * @brief 控制律可行性、跨周期状态与障碍投影测试。
 *
 * 本文件仅验证控制逻辑，不代替实机实验，也不构成整机碰撞安全认证。
 */

#include <tase_hcf_rcbf/certificate.h>
#include <tase_hcf_rcbf/torque_limits.h>

#include <iostream>
#include <stdexcept>
using namespace tase_hcf_rcbf;
void require(bool ok, const char* message) {
  if (!ok)
    throw std::runtime_error(message);
}
bool close(double a, double b, double tol = 1e-9) {
  return std::abs(a - b) <= tol;
}
int main() {
  Config c;
  Input i;
  i.lower = -1;
  i.upper = 1;
  i.dt = .001;
  i.dqd = .5;
  State s;
  auto o = step(c, i, s);
  require(o.valid && close(o.k_used, .5) && close(o.command_accel, 50) && close(o.dhat, 0),
          "HCF initialization");
  require(close(o.observer_p, -c.observer_gain * i.velocity), "observer coordinate initialization");
  i.q = .1;
  o = step(c, i, s);
  require(o.valid && different(o.k_next, o.k_used), "filter telemetry must retain pre/post states");
  State dob;
  Input di;
  di.lower = -100;
  di.upper = 100;
  di.dt = .001;
  step(c, di, dob);
  for (int k = 1; k <= 1000; ++k) {
    di.velocity = .4 * k * di.dt;
    di.previous_nominal_accel = .4;
    o = step(c, di, dob);
    require(o.valid && std::abs(o.dhat) < 1e-10, "constant acceleration DOB bias");
  }
  dob = State{};
  di.velocity = 0;
  step(c, di, dob);
  for (int k = 1; k <= 1000; ++k) {
    di.velocity = 1.1 * k * di.dt;
    o = step(c, di, dob);
  }
  require(close(o.dhat, .7 * (1 - std::exp(-10.)), 1e-10), "DOB constant disturbance response");
  c.mode = Mode::RCBF;
  c.physical_accel_bound = .5;
  i = Input{};
  i.dt = .001;
  i.lower = -1;
  i.upper = 1;
  i.q = .9;
  i.qd = 1.8;
  i.accel_min = -20;
  i.accel_max = 20;
  o = step(c, i, s);
  require(o.valid && o.nominal_outside_actuator && o.safety_filter_active &&
              !o.actuator_projection_changed && !o.actuator_constraint_active &&
              close(o.command_accel, 9.5),
          "RCBF false saturation regression");
  i.lower = -10;
  i.upper = 10;
  i.q = 0;
  i.qd = 1;
  i.accel_min = -1;
  i.accel_max = 1;
  o = step(c, i, s);
  require(o.valid && !o.safety_filter_active && o.actuator_projection_changed &&
              o.actuator_constraint_active && close(o.command_accel, 1),
          "actuator projection must be distinct");
  i.lower = -1;
  i.upper = 1;
  i.accel_min = -100;
  i.accel_max = 100;
  c.velocity_error_bound = .02;
  for (int qi = -8; qi <= 8; ++qi)
    for (int vi = -4; vi <= 4; ++vi) {
      i.q = .1 * qi;
      i.velocity = .1 * vi;
      o = step(c, i, s);
      if (!o.valid)
        continue;
      for (double b : {-.02, .02})
        for (double d : {-.5, .5}) {
          const double v = i.velocity - b;
          require(o.command_accel + d + 20 * (v - i.dlower) + 100 * (i.q - i.lower) >= -1e-9,
                  "robust lower HOCBF");
          require(-o.command_accel - d + 20 * (i.dupper - v) + 100 * (i.upper - i.q) >= -1e-9,
                  "robust upper HOCBF");
        }
    }
  const State before = s;
  i.accel_min = 200;
  i.accel_max = 300;
  o = step(c, i, s);
  require(!o.valid && !o.qp_feasible && s.k == before.k && s.dhat == before.dhat,
          "infeasible QP must not advance state");
  c.mode = Mode::HCF;
  i.accel_min = -100;
  i.accel_max = 100;
  i.lower = -.01;
  i.upper = .01;
  i.q = 0;
  o = step(c, i, s);
  require(!o.valid && !o.qp_feasible, "control-sharing failure");
  c.eta1 = 0;
  o = step(c, i, s);
  require(!o.valid, "invalid gain");
  c.eta1 = 5;
  auto a = torqueInterval(.5, 12, 1000, .001);
  require(a.valid && close(a.lower, -.5) && close(a.upper, 1.5), "Nm/s conversion");
  require(close(boundedHold(10, .5, 12, 1000, .001), 1.5), "rate clipping");
  require(close(boundedHold(0, 13, 12, 1000, .001), 12), "out-of-limit recovery");
  Disturbance d;
  d.enabled = true;
  d.profile = "case2";
  VelocityBias b;
  b.enabled = true;
  for (int k = 0; k <= 50000; ++k) {
    const double t = k * .001;
    require(std::abs(disturbanceValue(d, t)) <= disturbanceBound(d) + 1e-9, "disturbance bound");
    const auto v = biasSample(b, t);
    require(std::abs(v.value) <= biasBound(b) + 1e-9 &&
                std::abs(v.derivative) <= biasDerivativeBound(b) + 1e-9,
            "bias bounds");
  }
  const double t = 1.25, h = 1e-6;
  require(close((biasSample(b, t + h).value - biasSample(b, t - h).value) / (2 * h),
                biasSample(b, t).derivative, 1e-9),
          "bias derivative");
  const double physical_v = .3;
  const auto bv = biasSample(b, t);
  require(close((physical_v + bv.value) - bv.value, physical_v), "transformed d1=-bias identity");
  CertificateConfig cc;
  cc.R = 1;
  cc.L1 = 1;
  cc.K1 = 10;
  cc.P2 = 1.1;
  cc.mu1 = 0;
  cc.mu2 = 1.1;
  Harmonic low{-.5, .4, .5, false}, up{.6, .1, 1, true};
  auto cert = checkCertificate(c, cc, low, up);
  require(cert.available && cert.algebra_ok && close(cert.r1, .05), "n=2 certificate algebra");
  c.p1 = 2;
  c.p2 = 3;
  cert = checkCertificate(c, cc, low, up);
  require(close(cert.r1, .1) && close(cert.r2, .15), "p must multiply expansion radius");
  require(close(compositeH(.2, .1, .3, .1), .2 + .1 * std::exp(.2) - .02), "composite barrier");
  cc.mu2 = -1;
  require(!checkCertificate(c, cc, low, up).available, "unknown residual cannot certify");
  std::cout << "PASS: HCF/DOB, robust RCBF, velocity-bias transformation, flags, torque intervals, "
               "certificate checks\n";
}
