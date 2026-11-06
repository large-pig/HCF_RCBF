/**
 * @file test_commissioning.cpp
 * @brief 实机启动与运行状态阈值测试。
 *
 * 本文件仅验证控制逻辑，不代替实机实验，也不构成整机碰撞安全认证。
 */

#include <tase_hcf_rcbf/commissioning.h>

#include <iostream>
#include <limits>
#include <stdexcept>
using namespace tase_hcf_rcbf;
void require(bool ok, const char* msg) {
  if (!ok)
    throw std::runtime_error(msg);
}
int main() {
  const Harmonic h{.3, .05, .2, false};
  for (double t : {-1., 0., 50., 51.}) {
    const auto a = commissioningReference(h, t, 50, 5);
    require(a.value == .3 && a.derivative == 0 && a.second == 0, "C2 endpoint");
  }
  const double e = 1e-5;
  for (double t : {.1, 2.5, 4.9, 5., 15., 45., 47.5, 49.9}) {
    const auto a = commissioningReference(h, t, 50, 5), p = commissioningReference(h, t + e, 50, 5),
               m = commissioningReference(h, t - e, 50, 5);
    require(std::abs(a.derivative - (p.value - m.value) / (2 * e)) < 1e-7,
            "reference first derivative");
    require(std::abs(a.second - (p.derivative - m.derivative) / (2 * e)) < 1e-7,
            "reference second derivative");
  }
  CommissioningGuard g;
  g.enabled = true;
  std::array<double, 7> q{}, dq{}, start{};
  require(validGuard(g) && commissioningInitialOk(g, q, dq, 4, 0, false),
          "initial conditions accepted");
  q[4] = .03;
  require(!commissioningInitialOk(g, q, dq, 4, 0, false), "initial position rejected");
  require(commissioningInitialOk(g, q, dq, 4, 0, true), "hold at current position");
  q.fill(0);
  dq[0] = .03;
  require(!commissioningInitialOk(g, q, dq, 4, 0, true), "initial speed rejected");
  dq.fill(0);
  q[4] = .21;
  require(!commissioningMotionOk(g, q, dq, start, 4, 0, false), "excursion rejected");
  q.fill(0);
  q[0] = .041;
  require(!commissioningMotionOk(g, q, dq, start, 4, 0, false), "other joint drift rejected");
  q.fill(0);
  dq[4] = .201;
  require(!commissioningMotionOk(g, q, dq, start, 4, 0, false), "overspeed rejected");
  g.controlled_joint_excursion_enabled = false;
  g.controlled_joint_speed_enabled = false;
  q[4] = .8;
  dq[4] = .5;
  require(commissioningMotionOk(g, q, dq, start, 4, 0, false), "controlled joint motion opt-out");
  require(!commissioningInitialOk(g, q, dq, 4, 0, false),
          "initial checks unaffected by motion opt-out");
  require(!commissioningMotionOk(g, q, dq, start, 4, 0, true),
          "holding checks unaffected by motion opt-out");
  dq.fill(0);
  q[0] = .041;
  require(!commissioningMotionOk(g, q, dq, start, 4, 0, false),
          "other drift unaffected by motion opt-out");
  q[0] = 0;
  dq[0] = .201;
  require(!commissioningMotionOk(g, q, dq, start, 4, 0, false),
          "other speed unaffected by motion opt-out");
  dq[0] = 0;
  q[4] = std::numeric_limits<double>::quiet_NaN();
  require(!commissioningMotionOk(g, q, dq, start, 4, 0, false),
          "nonfinite controlled position still rejected");
  q[4] = 0;
  dq[4] = std::numeric_limits<double>::quiet_NaN();
  require(!commissioningMotionOk(g, q, dq, start, 4, 0, false),
          "nonfinite controlled velocity still rejected");
  // 仅为理想模型闭环回归测试，不作为实机验证结果。
  Config c;
  State s;
  double pos = 0, v = 0, previous = 0, minh = 1;
  for (int n = 0; n < 52000; ++n) {
    double t = n * .001;
    auto a = commissioningReference({0, .05, .2, false}, t, 50, 5);
    Input i;
    i.q = pos;
    i.velocity = v;
    i.qd = a.value;
    i.dqd = a.derivative;
    i.ddqd = a.second;
    i.lower = -.4;
    i.upper = .4;
    i.dt = .001;
    i.previous_nominal_accel = previous;
    auto o = step(c, i, s);
    require(o.valid, "calibration HCF feasible in ideal regression");
    previous = o.command_accel;
    v += .001 * previous;
    pos += .001 * v;
    minh = std::min(minh, .4 - std::abs(pos));
    require(std::abs(v) < .2 && std::abs(pos) < .2, "ideal trajectory inside commissioning guard");
  }
  require(minh > .3, "ideal position margin");
  std::cout
      << "PASS: reference derivatives, guard rejection, 52 s ideal commissioning regression\n";
}
