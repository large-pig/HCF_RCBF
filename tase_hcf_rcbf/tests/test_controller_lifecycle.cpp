/**
 * @file test_controller_lifecycle.cpp
 * @brief ROS 控制器初始化、控制状态及实验生命周期测试。
 *
 * 本文件仅验证控制逻辑，不代替实机实验，也不构成整机碰撞安全认证。
 */

#include <tase_hcf_rcbf/controller.h>

#include <iostream>
#include <stdexcept>

namespace tase_hcf_rcbf {
struct ControllerTestAccess {
  static void attach(JointComparisonController& c, franka_hw::FrankaModelInterface& m,
                     franka_hw::FrankaStateInterface& s,
                     hardware_interface::EffortJointInterface& e) {
    c.model_.reset(new franka_hw::FrankaModelHandle(m.getHandle("panda_model")));
    c.robot_.reset(new franka_hw::FrankaStateHandle(s.getHandle("panda_robot")));
    for (int j = 1; j <= 7; ++j)
      c.joints_.push_back(e.getHandle("panda_joint" + std::to_string(j)));
    c.limit_ = {{87, 87, 87, 87, 12, 12, 12}};
    c.rate_.fill(999.999);
  }
  static void experiment(JointComparisonController& c, bool rcbf) {
    c.hold_only_ = false;
    c.cfg_.mode = rcbf ? Mode::RCBF : Mode::HCF;
    c.cfg_.physical_accel_bound = 1.1;
  }
  static int fault(const JointComparisonController& c) { return c.fault_; }
  static bool initialized(const JointComparisonController& c) { return c.core_.initialized; }
  static bool initialOk(const JointComparisonController& c) {
    return c.initial_performed_ && c.initial_ok_;
  }
  static double holdingTarget(const JointComparisonController& c, int j) { return c.hold_[j]; }
  static void injection(JointComparisonController& c) {
    commissioning(c);
    c.allow_controlled_injection_ = true;
    c.cfg_.p1 = c.cfg_.p2 = 10;
    c.disturbance_.enabled = true;
    c.disturbance_.profile = "sine";
    c.disturbance_.units = "acceleration";
    c.disturbance_.amplitude = 4;
    c.disturbance_.omega = .5;
    c.disturbance_.ramp_s = 5;
  }
  static bool finished(const JointComparisonController& c) { return c.finished_; }
  static void duration(JointComparisonController& c, double x) { c.duration_ = x; }
  static void narrow(JointComparisonController& c) {
    c.lower_ = {-.01, 0, 0, false};
    c.upper_ = {.01, 0, 0, true};
  }
  static void commissioning(JointComparisonController& c) {
    c.guard_.enabled = true;
    c.reference_ = {0, .05, .2, false};
    c.reference_ramp_s_ = 5;
    c.lower_ = {-.4, 0, 0, false};
    c.upper_ = {.4, 0, 0, true};
  }
  static void moving(JointComparisonController& c, bool rcbf) {
    experiment(c, rcbf);
    c.cfg_.physical_accel_bound = 2;
    c.cfg_.p1 = c.cfg_.p2 = 10;
    c.cfg_.kp = 1600;
    c.cfg_.kd = 320;
    c.cfg_.k1 = c.cfg_.k2 = 30;
    c.guard_.enabled = true;
    c.reference_ = {0, 1, .5, false};
    c.reference_ramp_s_ = 5;
    c.guard_.controlled_joint_excursion_enabled = false;
    c.guard_.controlled_joint_speed_enabled = false;
    c.lower_ = {-.5, .4, .5, false};
    c.upper_ = {.6, .1, 1, true};
    c.allow_controlled_injection_ = true;
    c.waveform_name_ = "PERIODIC";
    c.waveform_ = InjectionWaveform::Periodic;
    c.disturbance_.enabled = true;
    c.disturbance_.profile = "sine";
    c.disturbance_.amplitude = 2;
    c.disturbance_.omega = .5;
    c.disturbance_.ramp_s = 5;
    c.hold_kp_ = 40;
    c.hold_kd_ = 8;
  }
};
}  // namespace tase_hcf_rcbf
class FakeModel : public franka_hw::ModelBase {
 public:
  double diagonal = .025;
  std::array<double, 16> pose(franka::Frame, const std::array<double, 7>&,
                              const std::array<double, 16>&,
                              const std::array<double, 16>&) const override {
    return {};
  }
  std::array<double, 42> bodyJacobian(franka::Frame, const std::array<double, 7>&,
                                      const std::array<double, 16>&,
                                      const std::array<double, 16>&) const override {
    return {};
  }
  std::array<double, 42> zeroJacobian(franka::Frame, const std::array<double, 7>&,
                                      const std::array<double, 16>&,
                                      const std::array<double, 16>&) const override {
    return {};
  }
  std::array<double, 49> mass(const std::array<double, 7>&, const std::array<double, 9>&, double,
                              const std::array<double, 3>&) const override {
    std::array<double, 49> a{};
    for (int j = 0; j < 7; ++j)
      a[j * 8] = diagonal;
    return a;
  }
  std::array<double, 7> coriolis(const std::array<double, 7>&, const std::array<double, 7>&,
                                 const std::array<double, 9>&, double,
                                 const std::array<double, 3>&) const override {
    return {};
  }
  std::array<double, 7> gravity(const std::array<double, 7>&, double, const std::array<double, 3>&,
                                const std::array<double, 3>&) const override {
    return {};
  }
};
struct Fixture {
  franka::RobotState state{};
  FakeModel model;
  std::array<double, 7> sent{};
  franka_hw::FrankaModelInterface models;
  franka_hw::FrankaStateInterface states;
  hardware_interface::EffortJointInterface efforts;
  tase_hcf_rcbf::JointComparisonController controller;
  Fixture() {
    state.q = {{0, -.5, 0, -1.5, 0, 1.2, 0}};
    models.registerHandle(franka_hw::FrankaModelHandle("panda_model", model, state));
    states.registerHandle(franka_hw::FrankaStateHandle("panda_robot", state));
    for (int j = 0; j < 7; ++j) {
      hardware_interface::JointStateHandle h("panda_joint" + std::to_string(j + 1), &state.q[j],
                                             &state.dq[j], &state.tau_J[j]);
      efforts.registerHandle(hardware_interface::JointHandle(h, &sent[j]));
    }
    tase_hcf_rcbf::ControllerTestAccess::attach(controller, models, states, efforts);
  }
  void start() { controller.starting(time(10)); }
  static ros::Time time(double v) {
    ros::Time t;
    t.fromSec(v);
    return t;
  }
  void update(double t, double dt) { controller.update(time(10 + t), ros::Duration(dt)); }
};
void require(bool ok, const char* message) {
  if (!ok)
    throw std::runtime_error(message);
}
int main() {
  using A = tase_hcf_rcbf::ControllerTestAccess;
  {
    Fixture f;
    f.start();
    f.update(0, 0);
    require(!A::fault(f.controller), "startup zero-period reset");
    f.update(.001, .001);
    require(!A::fault(f.controller) && !A::initialized(f.controller),
            "default hold must not run HCF");
  }
  for (bool rcbf : {false, true}) {
    Fixture f;
    A::experiment(f.controller, rcbf);
    f.start();
    f.update(0, 0);
    f.update(.001, .001);
    require(!A::fault(f.controller) && A::initialOk(f.controller), "first experiment cycle");
    for (double x : f.sent)
      require(std::isfinite(x), "finite torque");
    f.update(.002, 0);
    require(A::fault(f.controller) == 1, "later zero period must latch timing fault");
  }
  {
    Fixture f;
    A::experiment(f.controller, false);
    A::duration(f.controller, .002);
    f.start();
    f.update(.001, .001);
    const double target4 = A::holdingTarget(f.controller, 3);
    f.state.q[3] -= .01;
    f.state.q[4] += .001;
    f.update(.002, .001);
    require(A::finished(f.controller) && !A::fault(f.controller), "normal completion hold");
    require(A::holdingTarget(f.controller, 3) == target4,
            "other joint holding target must survive completion");
    require(A::holdingTarget(f.controller, 4) == f.state.q[4],
            "controlled joint captures final position");
    f.update(.003, 0);
    require(A::fault(f.controller) == 1 && A::holdingTarget(f.controller, 3) == target4,
            "other joint target must survive fault too");
  }
  {
    Fixture f;
    A::experiment(f.controller, false);
    A::narrow(f.controller);
    f.start();
    f.update(.001, .001);
    require(A::fault(f.controller) == 3, "infeasible upper QP must latch");
  }
  {
    Fixture f;
    A::experiment(f.controller, false);
    f.start();
    f.model.diagonal = 0;
    f.update(.001, .001);
    require(A::fault(f.controller) == 2, "invalid model must latch");
    for (double x : f.sent)
      require(std::isfinite(x), "invalid model bounded holding fallback");
  }
  {
    Fixture f;
    A::commissioning(f.controller);
    A::experiment(f.controller, false);
    f.state.q[4] = .03;
    f.start();
    require(A::fault(f.controller) == 9, "commissioning initial offset rejected");
  }
  {
    Fixture f;
    A::commissioning(f.controller);
    A::experiment(f.controller, false);
    f.start();
    f.update(.001, .001);
    require(!A::fault(f.controller), "commissioning start");
    f.state.dq[1] = .21;
    f.update(.002, .001);
    require(A::fault(f.controller) == 10, "commissioning overspeed latched");
  }
  {
    Fixture f;
    A::experiment(f.controller, false);
    A::injection(f.controller);
    f.start();
    f.model.diagonal = .1;
    f.update(3.14, .001);
    require(A::fault(f.controller) == 11, "injection torque cap fault latched");
  }
  for (bool rcbf : {false, true}) {
    Fixture f;
    A::moving(f.controller, rcbf);
    f.start();
    f.update(.001, .001);
    require(!A::fault(f.controller), "moving task initial sample");
    const double target4 = A::holdingTarget(f.controller, 3),
                 target5 = A::holdingTarget(f.controller, 4);
    f.state.q[4] = -.114;
    f.state.dq[4] = .01;
    require(f.state.q[4] < -.5 + .4 * std::sin(.5 * 27.75),
            "fixture crosses experimental virtual lower boundary");
    f.update(27.75, .001);
    require(!A::fault(f.controller) && !A::finished(f.controller),
            "virtual crossing does not latch fault 12 or end trajectory");
    require(A::holdingTarget(f.controller, 4) == target5,
            "virtual crossing does not capture holding target");
    for (double x : f.sent)
      require(std::isfinite(x) && std::abs(x) <= 12, "crossing still uses bounded torque");
    f.state.q[3] -= .041;
    f.update(27.751, .001);
    require(A::fault(f.controller) == 10,
            "other-joint drift guard remains active after virtual crossing");
    require(A::holdingTarget(f.controller, 3) == target4,
            "other six holding targets survive real guard fault");
  }
  for (bool rcbf : {false, true}) {
    Fixture f;
    A::moving(f.controller, rcbf);
    f.start();
    f.update(.001, .001);
    f.state.q[4] = .3;
    f.state.dq[4] = .3;
    f.update(12, .001);
    require(!A::fault(f.controller), "moving joint-5 excursion and speed checks disabled");
    f.state.dq[4] = 0;
    f.update(50, .001);
    require(!A::fault(f.controller) && A::finished(f.controller),
            "finish captures joint-5 target away from initial pose");
    require(A::holdingTarget(f.controller, 4) == .3, "finish captures only controlled joint");
    f.state.q[4] = .341;
    f.update(50.001, .001);
    require(A::fault(f.controller) == 10, "joint-5 holding drift check remains active");
  }
  {
    Fixture f;
    A::moving(f.controller, true);
    f.start();
    f.update(.001, .001);
    f.state.dq[0] = .201;
    f.update(.002, .001);
    require(A::fault(f.controller) == 10, "other-joint speed guard remains active");
  }
  {
    Fixture f;
    A::moving(f.controller, true);
    f.start();
    f.update(.001, .001);
    f.update(50, .001);
    f.state.dq[4] = .201;
    f.update(50.001, .001);
    require(A::fault(f.controller) == 10, "joint-5 holding speed check remains active");
  }
  std::cout << "PASS: real ROS/Franka handle lifecycle, no ROS master or robot connected\n";
}
