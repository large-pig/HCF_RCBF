/**
 * @file controller.h
 * @brief Franka Panda 的 HCF / RCBF ROS 控制器接口。
 *
 * 控制器申请七个关节的力矩控制句柄：第 5 关节执行选定的安全控制律，
 * 其余关节采用位置保持控制。
 */

#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tase_hcf_rcbf/TrialSample.h>

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

#include "certificate.h"
#include "commissioning.h"
#include "controlled_injection.h"
#include "shared_compensation.h"
#include "torque_limits.h"

namespace tase_hcf_rcbf {
/**
 * 面向 Franka 硬件接口的 ROS 控制器插件及运行状态机。
 * init() 校验参数；starting() 初始化采样与控制状态；
 * update() 执行实时控制，避免阻塞式日志与外部 I/O。
 */
class JointComparisonController
    : public controller_interface::MultiInterfaceController<
          franka_hw::FrankaModelInterface, franka_hw::FrankaStateInterface,
          hardware_interface::EffortJointInterface> {
 public:
  bool init(hardware_interface::RobotHW*, ros::NodeHandle&) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration&) override;

 private:
  friend struct ControllerTestAccess;
  void captureHold(const franka::RobotState&);
  void latch(int, const franka::RobotState&);
  void publishConfig(ros::NodeHandle&);
  // Franka 模型/状态/力矩接口及实验采样发布器。
  std::unique_ptr<franka_hw::FrankaModelHandle> model_;
  std::unique_ptr<franka_hw::FrankaStateHandle> robot_;
  std::vector<hardware_interface::JointHandle> joints_;
  std::unique_ptr<realtime_tools::RealtimePublisher<TrialSample>> sample_pub_;
  ros::Publisher config_pub_;
  // 控制算法、扰动配置及启动/运行状态检查。
  Config cfg_;
  State core_;
  CertificateConfig cert_cfg_;
  Certificate cert_;
  Harmonic reference_{0, 1, .5, false};
  Harmonic lower_{-.5, .4, .5, false};
  Harmonic upper_{.6, .1, 1, true};
  Disturbance disturbance_;
  VelocityBias bias_;
  FrictionModel friction_;
  SharedCompensation shared_;
  bool allow_controlled_injection_ = false;
  bool d1_injection_only_budget_ = false;
  InjectionWaveform waveform_ = InjectionWaveform::Invalid;
  std::string waveform_name_;
  CommissioningGuard guard_;
  double reference_ramp_s_ = 0;
  // 各关节保持目标、执行器幅值及变化率限制。
  std::array<double, 7> start_pose_{};
  std::array<double, 7> hold_{};
  std::array<double, 7> previous_{};
  std::array<double, 7> limit_{};
  std::array<double, 7> rate_{};
  ros::Time start_;
  int joint_ = 4, fault_ = 0;
  bool hold_only_ = true;
  bool reset_pending_ = true;
  bool finished_ = false;
  bool initial_performed_ = false;
  bool initial_ok_ = false;
  bool cert_initial_ok_ = false;
  bool enforce_certificate_ = false;
  double hold_kp_ = 20;
  double hold_kd_ = 4;
  double duration_ = 50;
  double max_dt_ = .01;
  double residual_bound_ = -1;
  double mass_min_bound_ = -1;
  double previous_accel_ = 0;
  double transformed_bound_ = -1;
  double previous_update_us_ = 0;
  uint64_t cycle_ = 0;
  uint64_t dropped_ = 0;
  uint64_t previous_timed_cycle_ = 0;
  std::string arm_ = "panda";
  std::string mode_ = "HCF";
  std::string trial_id_ = "unset";
};
}  // 命名空间 tase_hcf_rcbf
