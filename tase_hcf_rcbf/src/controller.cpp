/**
 * @file controller.cpp
 * @brief Franka Panda 第 5 关节 HCF / RCBF 实时控制器。
 *
 * 实现顺序：检查运行配置，获取机器人状态与动力学模型，计算安全控制指令，
 * 将加速度转换为关节力矩并执行约束，最后通过非阻塞方式发布采样数据。
 * rosbag 录制在控制循环以外进行，避免实时回调等待磁盘写入。
 */

#include <pluginlib/class_list_macros.h>
#include <tase_hcf_rcbf/controller.h>

#include <chrono>
#include <sstream>

namespace tase_hcf_rcbf {
namespace {
double holdCommand(double target, double q, double dq, double C, double kp, double kd,
                   double previous) {
  const double a = kp * (target - q) - kd * dq + C;
  return finite(a) ? a : (finite(previous) ? previous : 0);
}
double elapsedUs(const std::chrono::steady_clock::time_point& begin) {
  return std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - begin)
      .count();
}
}  // namespace

/**
 * @brief 初始化控制器并检查运行参数。
 *
 * 只在明确启用实机且关键参数合法时获取关节力矩接口；
 * 参数不合法时直接返回失败，避免使用不完整配置运行控制器。
 */
bool JointComparisonController::init(hardware_interface::RobotHW* hw, ros::NodeHandle& nh) {
  // 实机控制必须显式授权，未启用时不申请关节力矩资源。
  bool enabled = false;
  nh.param("enable_hardware", enabled, false);
  if (!enabled) {
    return false;
  }
  // 控制方法及受控关节索引由 launch 配置确定；
  // 数组索引从 0 开始，第 5 关节的索引为 4。
  nh.param("arm_id", arm_, arm_);
  nh.param("mode", mode_, mode_);
  nh.param("trial_id", trial_id_, trial_id_);
  nh.param("controlled_joint_index", joint_, joint_);
  nh.param("hold_only", hold_only_, hold_only_);
  if (mode_ == "HCF") {
    cfg_.mode = Mode::HCF;
  } else if (mode_ == "RCBF") {
    cfg_.mode = Mode::RCBF;
  } else {
    return false;
  }
  if (joint_ < 0 || joint_ > 6) {
    return false;
  }
  // 从当前控制器的 ROS 参数命名空间读取配置，
  // 参数优先级包含实验 YAML 与 launch 文件覆盖值。
  // 参数包含障碍增益、动态参考、扰动注入、
  // 启动检查阈值、共用补偿及证书判据等。
  auto load_param = [&nh](const std::string& name, auto& value) { nh.param(name, value, value); };
  load_param("ke", cfg_.ke);
  load_param("eta1", cfg_.eta1);
  load_param("eta2", cfg_.eta2);
  load_param("p1", cfg_.p1);
  load_param("p2", cfg_.p2);
  load_param("tau_filter", cfg_.tau);
  load_param("rho", cfg_.rho);
  load_param("observer_gain", cfg_.observer_gain);
  load_param("kp", cfg_.kp);
  load_param("kd", cfg_.kd);
  load_param("rcbf_k1", cfg_.k1);
  load_param("rcbf_k2", cfg_.k2);
  load_param("hold_kp", hold_kp_);
  load_param("hold_kd", hold_kd_);
  load_param("run_duration", duration_);
  load_param("max_dt", max_dt_);
  load_param("reference/ramp_s", reference_ramp_s_);
  load_param("commissioning/enabled", guard_.enabled);
  load_param("commissioning/controlled_joint_excursion_enabled",
             guard_.controlled_joint_excursion_enabled);
  load_param("commissioning/controlled_joint_speed_enabled", guard_.controlled_joint_speed_enabled);
  load_param("commissioning/allow_controlled_injection", allow_controlled_injection_);
  load_param("d1_injection_only_budget", d1_injection_only_budget_);
  load_param("injection_comparison_waveform", waveform_name_);
  load_param("commissioning/initial_position_tolerance", guard_.initial_position_tolerance);
  load_param("commissioning/initial_speed", guard_.initial_speed);
  load_param("commissioning/excursion", guard_.excursion);
  load_param("commissioning/max_speed", guard_.max_speed);
  load_param("commissioning/other_joint_drift", guard_.other_joint_drift);
  load_param("reference/offset", reference_.offset);
  load_param("reference/amplitude", reference_.amplitude);
  load_param("reference/omega", reference_.omega);
  load_param("lower/offset", lower_.offset);
  load_param("lower/amplitude", lower_.amplitude);
  load_param("lower/omega", lower_.omega);
  load_param("upper/offset", upper_.offset);
  load_param("upper/amplitude", upper_.amplitude);
  load_param("upper/omega", upper_.omega);
  load_param("injection/enabled", disturbance_.enabled);
  load_param("injection/profile", disturbance_.profile);
  load_param("injection/units", disturbance_.units);
  load_param("injection/amplitude", disturbance_.amplitude);
  load_param("injection/omega", disturbance_.omega);
  load_param("injection/scale", disturbance_.scale);
  load_param("injection/ramp_s", disturbance_.ramp_s);
  load_param("velocity_bias/enabled", bias_.enabled);
  load_param("velocity_bias/amplitude", bias_.amplitude);
  load_param("velocity_bias/omega", bias_.omega);
  load_param("velocity_bias/ramp_s", bias_.ramp_s);
  load_param("friction/coulomb_nm", friction_.coulomb);
  load_param("friction/viscous_nms", friction_.viscous);
  load_param("friction/smooth_velocity", friction_.smooth_velocity);
  load_param("residual_accel_bound", residual_bound_);
  load_param("mass_min_for_torque_bound", mass_min_bound_);
  load_param("shared_compensation/enabled", shared_.enabled);
  load_param("shared_compensation/offset_nm", shared_.offset);
  load_param("shared_compensation/coulomb_nm", shared_.coulomb);
  load_param("shared_compensation/viscous_nms", shared_.viscous);
  load_param("shared_compensation/velocity_scale", shared_.velocity_scale);
  load_param("shared_compensation/scale", shared_.scale);
  load_param("shared_compensation/ramp_s", shared_.ramp_s);
  load_param("shared_compensation/cap_nm", shared_.cap);
  load_param("shared_compensation/rate_nm_s", shared_.rate);
  load_param("certificate/enforce", enforce_certificate_);
  load_param("certificate/R", cert_cfg_.R);
  load_param("certificate/nu", cert_cfg_.nu);
  load_param("certificate/L1", cert_cfg_.L1);
  load_param("certificate/K1", cert_cfg_.K1);
  load_param("certificate/P2", cert_cfg_.P2);
  // 实机启动前拒绝非有限参数和无效的扰动/轨迹配置。
  if (!validDisturbance(disturbance_) || !validBias(bias_) || !validFriction(friction_) ||
      !validHarmonic(reference_) || !validHarmonic(lower_) || !validHarmonic(upper_))
    return false;
  if (!shared_.valid()) {
    return false;
  }
  if (!finite(duration_) || duration_ <= 0 || !finite(max_dt_) || max_dt_ <= 0 ||
      !finite(hold_kp_) || hold_kp_ <= 0 || !finite(hold_kd_) || hold_kd_ <= 0 ||
      !finite(residual_bound_) || !finite(mass_min_bound_))
    return false;
  if (!validGuard(guard_) || !finite(reference_ramp_s_) || reference_ramp_s_ < 0 ||
      2 * reference_ramp_s_ > duration_)
    return false;
  // 最终三组扰动波形仅适用于与参数预算一致的实验条件，
  // 不允许任意修改参考轨迹、时长或扰动幅值后仍复用原证明界。
  const bool controlled_injection_ok =
      allow_controlled_injection_ && validControlledInjection(disturbance_, duration_);
  if (!waveform_name_.empty()) {
    waveform_ = parseWaveform(waveform_name_);
    if (waveform_ == InjectionWaveform::Invalid || d1_injection_only_budget_ ||
        !controlled_injection_ok || !guard_.enabled || hold_only_ || joint_ != 4 || bias_.enabled ||
        enforce_certificate_ || !disturbance_.enabled || disturbance_.amplitude != 2 ||
        disturbance_.omega != .5 || disturbance_.ramp_s != 5 ||
        !validMovingResidualBudget(cfg_.mode, residual_bound_, waveform_) || duration_ != 50 ||
        reference_.offset != 0 || reference_.amplitude != 1 || reference_.omega != .5 ||
        reference_ramp_s_ != 5 || guard_.controlled_joint_excursion_enabled ||
        guard_.controlled_joint_speed_enabled || lower_.offset != -.5 || lower_.amplitude != .4 ||
        lower_.omega != .5 || upper_.offset != .6 || upper_.amplitude != .1 || upper_.omega != 1) {
      return false;
    }
  }
  if ((!guard_.controlled_joint_excursion_enabled || !guard_.controlled_joint_speed_enabled) &&
      (waveform_ == InjectionWaveform::Invalid || joint_ != 4 || hold_only_)) {
    return false;
  }
  if (d1_injection_only_budget_ &&
      (!controlled_injection_ok || !guard_.enabled || hold_only_ || cfg_.mode != Mode::RCBF ||
       joint_ != 4 || bias_.enabled || enforce_certificate_ || !disturbance_.enabled ||
       disturbance_.amplitude != 2 || disturbance_.omega != .5 || disturbance_.ramp_s != 5 ||
       residual_bound_ != 0 || duration_ != 50 || reference_.offset != 0 ||
       reference_.amplitude != .15 || reference_.omega != .2 || reference_ramp_s_ != 5 ||
       lower_.offset != -.12 || upper_.offset != .12 || lower_.amplitude != 0 ||
       upper_.amplitude != 0)) {
    return false;
  }
  const bool sensitivity_budget =
      d1_injection_only_budget_ || waveform_ != InjectionWaveform::Invalid;
  if (allow_controlled_injection_ &&
      (!controlled_injection_ok || (!sensitivity_budget && residual_bound_ < 60))) {
    return false;
  }
  if (guard_.enabled && ((guard_.controlled_joint_excursion_enabled &&
                          std::abs(reference_.amplitude) >= guard_.excursion) ||
                         (disturbance_.enabled && !controlled_injection_ok) || bias_.enabled ||
                         enforce_certificate_)) {
    return false;
  }
  // 软件注入扰动的幅值已知，但并不代表真实机器人全部
  // 未建模动力学和接触外力都受该幅值限制。剩余扰动预算
  // 单独配置，只有满足适用条件时才与注入界合并。
  double injected_bound = disturbanceBound(disturbance_);
  bool physical_bound_available = residual_bound_ >= 0;
  if (disturbance_.enabled && disturbance_.units == "torque") {
    physical_bound_available = physical_bound_available && mass_min_bound_ > 0;
    if (mass_min_bound_ > 0)
      injected_bound /= mass_min_bound_;
  }
  cfg_.physical_accel_bound = physical_bound_available ? residual_bound_ + injected_bound : -1;
  cfg_.velocity_error_bound = biasBound(bias_);
  transformed_bound_ =
      physical_bound_available ? cfg_.physical_accel_bound + biasDerivativeBound(bias_) : -1;
  // 原位保持模式下不调用 RCBF 轨迹控制计算；
  // 这项验证分支并不意味着允许未配置的轨迹运行。
  Config checked = cfg_;
  if (hold_only_ && checked.mode == Mode::RCBF && checked.physical_accel_bound < 0)
    checked.physical_accel_bound = 0;
  if (!validConfig(checked)) {
    return false;
  }
  if (!hold_only_ && cfg_.mode == Mode::RCBF && !physical_bound_available) {
    return false;
  }
  cert_cfg_.mu1 = biasBound(bias_);
  cert_cfg_.mu2 = transformed_bound_;
  Config cert_config = cfg_;
  cert_config.mode = Mode::HCF;
  cert_ = checkCertificate(cert_config, cert_cfg_, lower_, upper_);
  if (cfg_.mode == Mode::HCF && enforce_certificate_ && !hold_only_ && !cert_.algebra_ok) {
    return false;
  }
  const double min_gap =
      upper_.offset - std::abs(upper_.amplitude) - lower_.offset - std::abs(lower_.amplitude);
  if (min_gap <= 0) {
    return false;
  }
  // 为七个关节统一配置力矩幅值和变化率限制；
  // 非受控关节的位置保持力矩也采用相同的限制逻辑。
  std::vector<double> limits, rates;
  if (!nh.getParam("torque_limits", limits) || !nh.getParam("torque_rates", rates) ||
      limits.size() != 7 || rates.size() != 7)
    return false;
  for (int j = 0; j < 7; ++j) {
    if (!finite(limits[j]) || limits[j] <= 0 || !finite(rates[j]) || rates[j] <= 0)
      return false;
    limit_[j] = limits[j];
    rate_[j] = rates[j];
  }
  // 使用 franka_ros 的模型、状态和力矩控制接口；
  // 不修改机器人固件或底层重力补偿实现。
  auto* mi = hw->get<franka_hw::FrankaModelInterface>();
  auto* si = hw->get<franka_hw::FrankaStateInterface>();
  auto* ei = hw->get<hardware_interface::EffortJointInterface>();
  if (!mi || !si || !ei) {
    return false;
  }
  try {
    model_.reset(new franka_hw::FrankaModelHandle(mi->getHandle(arm_ + "_model")));
    robot_.reset(new franka_hw::FrankaStateHandle(si->getHandle(arm_ + "_robot")));
    for (int j = 1; j <= 7; ++j)
      joints_.push_back(ei->getHandle(arm_ + "_joint" + std::to_string(j)));
  } catch (const hardware_interface::HardwareInterfaceException&) {
    return false;
  }
  // 实验数据采用实时安全的发布方式，rosbag 由独立节点运行，
  // 控制计算不会等待订阅者处理或写入文件。
  sample_pub_.reset(new realtime_tools::RealtimePublisher<TrialSample>(nh, "sample", 10));
  sample_pub_->msg_.trial_id = trial_id_;
  sample_pub_->msg_.scenario = disturbance_.enabled ? disturbance_.profile : "none";
  sample_pub_->msg_.velocity_bias_enabled = bias_.enabled;
  if (waveform_ != InjectionWaveform::Invalid)
    sample_pub_->msg_.scenario = waveform_name_;
  config_pub_ = nh.advertise<std_msgs::String>("configuration", 1, true);
  publishConfig(nh);
  return true;
}

/** Publish the effective ROS parameters once for trial reproducibility. */
void JointComparisonController::publishConfig(ros::NodeHandle& nh) {
  std_msgs::String msg;
  std::ostringstream s;
  s.precision(17);
  s << "schema=2;trial_id=" << trial_id_ << ";arm_id=" << arm_ << ";joint_index=" << joint_
    << ";mode=" << mode_ << ";hold_only=" << hold_only_ << ";d1_semantics="
    << (bias_.enabled ? "minus_velocity_bias_in_transformed_state" : "zero_physical_kinematics")
    << ";torque_interface=gravity_compensated;coupling=bounded_residual_assumption"
    << ";physical_accel_bound=" << cfg_.physical_accel_bound
    << ";transformed_accel_bound=" << transformed_bound_
    << ";certificate_algebra_ok=" << cert_.algebra_ok
    << ";certificate_is_not_physical_bound_validation=1"
    << ";budget_policy="
    << (waveform_ != InjectionWaveform::Invalid && residual_bound_ > 0
            ? "assumed_residual_plus_injection_sensitivity"
            : ((d1_injection_only_budget_ || waveform_ != InjectionWaveform::Invalid)
                   ? "injection_only_sensitivity"
                   : "configured_residual_plus_injection"))
    << ";injection_comparison_waveform=" << waveform_name_
    << ";virtual_boundary_policy=monitor_only"
    << ";controlled_joint_motion_excursion_check=" << guard_.controlled_joint_excursion_enabled
    << ";controlled_joint_motion_speed_check=" << guard_.controlled_joint_speed_enabled;
  // 在每次实验开始时发布最终生效参数，便于复现实验条件。
  XmlRpc::XmlRpcValue params;
  if (nh.getParam("", params))
    s << ";ros_parameters=" << params.toXml();
  msg.data = s.str();
  config_pub_.publish(msg);
}

/** Reset controller state and check the robot pose at trial start. */
void JointComparisonController::starting(const ros::Time& time) {
  const auto r = robot_->getRobotState();
  hold_ = r.q;
  start_pose_ = r.q;
  previous_ = r.tau_J_d;
  start_ = time;
  core_ = State{};
  fault_ = 0;
  finished_ = false;
  reset_pending_ = true;
  initial_performed_ = false;
  initial_ok_ = false;
  cert_initial_ok_ = false;
  previous_accel_ = 0;
  previous_update_us_ = 0;
  cycle_ = 0;
  dropped_ = 0;
  previous_timed_cycle_ = 0;
  shared_.value = 0;
  for (int j = 0; j < 7; ++j)
    if (!finite(r.q[j]) || !finite(r.dq[j]) || !finite(previous_[j]) ||
        std::abs(previous_[j]) > limit_[j])
      latch(5, r);
  if (!commissioningInitialOk(guard_, r.q, r.dq, joint_, reference_.offset, hold_only_))
    latch(9, r);
}

/** Save the active joint pose as its holding target when motion ends. */
void JointComparisonController::captureHold(const franka::RobotState& r) {
  // 保持其余六个关节的初始目标位置；受控关节出现故障时
  // 切换为位置保持，不能将该处理视为经认证的安全停止。
  if (!hold_only_ && finite(r.q[joint_]))
    hold_[joint_] = r.q[joint_];
}

/** Latch the first fault and prevent further trajectory commands. */
void JointComparisonController::latch(int reason, const franka::RobotState& r) {
  if (!fault_) {
    fault_ = reason;
    captureHold(r);
  }
}

/**
 * @brief 执行一个实时控制周期。
 *
 * 本回调只进行状态读取、控制计算、力矩下发与非阻塞数据发布，
 * 不执行文件写入或控制台日志输出，避免磁盘和终端 I/O 影响控制周期。
 */
void JointComparisonController::update(const ros::Time& time, const ros::Duration& period) {
  const auto begin = std::chrono::steady_clock::now();
  const auto r = robot_->getRobotState();
  const double dt = period.toSec(), t = (time - start_).toSec();
  if (reset_pending_) {
    reset_pending_ = false;
    if (dt == 0 && !fault_) {
      for (int j = 0; j < 7; ++j)
        joints_[j].setCommand(previous_[j]);
      return;
    }
  }
  ++cycle_;
  // 利用完整七关节状态在线获得质量矩阵及科氏/离心项，
  // 受控关节使用惯量对角元 M55 进行加速度至力矩的转换。
  const auto M = model_->getMass();
  const auto C = model_->getCoriolis();
  const double m = M[7 * joint_ + joint_];
  std::array<double, 7> rate_reference{};
  for (int j = 0; j < 7; ++j)
    rate_reference[j] = finite(r.tau_J_d[j]) ? r.tau_J_d[j] : previous_[j];
  const bool timing = finite(dt) && dt > 0 && dt <= max_dt_ && finite(t) && t >= 0;
  bool model_ok = finite(m) && m > 1e-9;
  for (int j = 0; j < 7; ++j)
    model_ok =
        model_ok && finite(r.q[j]) && finite(r.dq[j]) && finite(C[j]) && finite(r.tau_J_d[j]);
  if (!timing) {
    latch(1, r);
  } else if (!model_ok) {
    latch(2, r);
  }
  // 到达设定的运动时长后切换至位置保持，
  // 启动状态检查同时监测其他六个关节的运动。
  if (!fault_ && !hold_only_ && !finished_ && t >= duration_) {
    finished_ = true;
    captureHold(r);
  }
  const bool holding = hold_only_ || finished_ || fault_;
  auto guard_targets = start_pose_;
  if (holding)
    guard_targets[joint_] = hold_[joint_];
  if (!commissioningMotionOk(guard_, r.q, r.dq, guard_targets, joint_, reference_.offset, holding))
    latch(10, r);
  bool running = !fault_ && !hold_only_ && !finished_;
  const auto ref = commissioningReference(reference_, t, duration_, reference_ramp_s_),
             low = sample(lower_, t), up = sample(upper_, t);
  // 动态边界用于计算和记录安全裕度，并非直接触发停止的硬阈值。
  // h1、h2 保存原始（未扩展）安全函数的有符号裕度。
  const auto b = running ? biasSample(bias_, t) : Sample{};
  // 速度偏差只施加在控制器观测量上，不会直接修改机械臂实际状态。
  const double velocity = r.dq[joint_] + b.value;
  double injection_signal =
      running ? (allow_controlled_injection_ ? controlledInjectionValue(disturbance_, t, duration_)
                                             : disturbanceValue(disturbance_, t))
              : 0;
  if (running && waveform_ != InjectionWaveform::Invalid)
    injection_signal = shapedInjectionValue(waveform_, disturbance_, t, duration_);
  // 加速度域扰动先乘对应的惯量对角元，再注入用户期望力矩。
  // 力矩域扰动本身单位为 N·m，无须再次换算。
  double injection = disturbance_.units == "acceleration" ? m * injection_signal : injection_signal;
  const double friction =
      frictionTorque(friction_, r.dq[joint_]) + shared_.update(r.dq[joint_], t, dt, running);
  if (!finite(injection) || !finite(friction)) {
    latch(5, r);
    running = false;
    injection = 0;
    injection_signal = 0;
  }
  if (running && allow_controlled_injection_ && std::abs(injection) > .12) {
    latch(11, r);
    running = false;
    injection = 0;
    injection_signal = 0;
  }
  if (running && disturbance_.enabled && disturbance_.units == "torque" && mass_min_bound_ > 0 &&
      m < mass_min_bound_)
    latch(8, r);
  // 组装控制器输入：当前关节状态、参考信号导数、动态边界，
  // 以及根据执行器限制得到的允许加速度区间。
  Input in;
  in.q = r.q[joint_];
  in.velocity = velocity;
  in.qd = ref.value;
  in.dqd = ref.derivative;
  in.ddqd = ref.second;
  in.lower = low.value;
  in.dlower = low.derivative;
  in.ddlower = low.second;
  in.upper = up.value;
  in.dupper = up.derivative;
  in.ddupper = up.second;
  in.dt = dt;
  in.previous_nominal_accel = previous_accel_;
  // 在安全控制计算前，将力矩幅值/变化率限制换算为
  // 受控关节可执行的加速度区间。
  if (timing && model_ok) {
    const auto a = torqueInterval(rate_reference[joint_], limit_[joint_], rate_[joint_], dt);
    if (!a.valid)
      latch(5, r);
    else {
      in.accel_min = (a.lower - C[joint_] - friction - injection) / m;
      in.accel_max = (a.upper - C[joint_] - friction - injection) / m;
    }
  }
  // HCF 先生成并滤波安全虚拟速度；RCBF 将名义加速度
  // 投影到障碍条件与执行器条件共同允许的区间。
  Output o;
  if (running && !fault_) {
    o = step(cfg_, in, core_);
    if (!o.valid)
      latch(3, r);
    if (!initial_performed_) {
      initial_performed_ = true;
      initial_ok_ = o.valid && o.initial_safe;
      if (!initial_ok_)
        latch(4, r);
      if (cfg_.mode == Mode::HCF && cert_.available) {
        cert_initial_ok_ = o.valid && compositeH(o.h1, cert_.r1, velocity, o.k_used) >= 0 &&
                           compositeH(o.h2, cert_.r2, velocity, o.k_used) >= 0;
      }
      if (cfg_.mode == Mode::HCF && enforce_certificate_ && !cert_initial_ok_)
        latch(6, r);
    }
    // 执行器限幅可能使名义安全条件不再成立；
    // 如果启用了证书约束，则在检查失败时退出轨迹运行状态。
    if (cfg_.mode == Mode::HCF && enforce_certificate_ && o.actuator_projection_changed)
      latch(7, r);
  }
  // 可选地监控 HCF 证书在原始安全裕度下是否满足；
  // 该监控不能代替对真实扰动界的实验验证。
  double H1 = 0, H2 = 0;
  bool certificate_monitor = false;
  if (cfg_.mode == Mode::HCF && cert_.available && o.valid) {
    H1 = compositeH(o.h1, cert_.r1, velocity, o.k_used);
    H2 = compositeH(o.h2, cert_.r2, velocity, o.k_used);
    certificate_monitor = H1 >= -1e-9 && H2 >= -1e-9 && std::abs(o.dhat) <= cert_cfg_.P2 + 1e-9 &&
                          std::abs(o.k_used) <= cert_cfg_.K1 + 1e-9 &&
                          std::abs(o.k_used - o.virtual_target) <= cfg_.tau * cert_cfg_.L1 + 1e-9;
    if (enforce_certificate_ && !certificate_monitor)
      latch(6, r);
  }
  running = running && !fault_;
  if (!running) {
    injection = 0;
    injection_signal = 0;
  }
  // 力矩指令使用在线科氏/离心项与等效惯量：
  // 重力补偿由 Franka 底层力矩接口完成，不在这里重复叠加。
  // 非受控关节采用位置保持控制器。
  // 将加速度指令（rad/s²）换算成请求力矩（N·m）：
  // τ = C5 + 模型补偿 + M55 × a + 注入力矩。
  // 控制层不再重复添加 G5。
  std::array<double, 7> requested{}, sent{};
  std::array<bool, 7> postclip{};
  for (int j = 0; j < 7; ++j) {
    requested[j] = (running && j == joint_) ? C[j] + friction + m * o.command_accel + injection
                                            : holdCommand(hold_[j], r.q[j], r.dq[j], C[j], hold_kp_,
                                                          hold_kd_, previous_[j]);
    if (!finite(requested[j])) {
      latch(5, r);
      requested[j] = finite(previous_[j]) ? previous_[j] : 0;
    }
  }
  if (fault_) {
    running = false;
    injection = 0;
    injection_signal = 0;
    for (int j = 0; j < 7; ++j)
      requested[j] = holdCommand(hold_[j], r.q[j], r.dq[j], C[j], hold_kp_, hold_kd_, previous_[j]);
  }
  // 调用 setCommand 前，始终执行用户层力矩幅值与变化率限制。
  const double limiting_dt = timing ? dt : (finite(dt) && dt > 0 ? std::min(dt, max_dt_) : 0);
  for (int j = 0; j < 7; ++j) {
    sent[j] = boundedHold(requested[j], rate_reference[j], limit_[j], rate_[j], limiting_dt);
    postclip[j] = different(sent[j], requested[j]);
    joints_[j].setCommand(sent[j]);
    previous_[j] = sent[j];
  }
  const double previous_accel_used = previous_accel_;
  previous_accel_ =
      model_ok && finite(friction) ? (sent[joint_] - C[joint_] - friction - injection) / m : 0;
  // 计算耗时在发布实验数据前结束计时，
  // 从而区分控制计算和通信/录制产生的开销。
  const double compute_us = elapsedUs(begin);
  // 实时采样采用非阻塞发布，控制周期不依赖订阅者响应；
  // 每次实验需要的控制与时间数据均可以按消息字段保存。
  // 保留六组实验所用的完整数据字段结构，包括
  // 实际/命令力矩、安全裕度和单周期计算时间。
  if (sample_pub_ && sample_pub_->trylock()) {
    auto& z = sample_pub_->msg_;
    z.header.stamp = time;
    z.schema_version = 2;
    z.cycle = cycle_;
    z.dropped_sample_pub_total = dropped_;
    z.phase = fault_ ? 3 : (hold_only_ ? 0 : (finished_ ? 2 : 1));
    z.mode = cfg_.mode == Mode::HCF ? 0 : 1;
    z.joint_index = joint_;
    z.fault_reason = fault_;
    z.time_s = t;
    z.run_duration_s = duration_;
    z.control_period_s = dt;
    z.core_valid = running && o.valid;
    z.qp_feasible = running && o.qp_feasible;
    z.initial_check_performed = initial_performed_;
    z.initial_check_ok = initial_ok_;
    z.top_filter_active = o.top_filter_active;
    z.safety_filter_active = o.safety_filter_active;
    z.nominal_outside_actuator = o.nominal_outside_actuator;
    z.actuator_projection_changed = o.actuator_projection_changed;
    z.actuator_constraint_active = o.actuator_constraint_active;
    z.post_clip_changed = postclip[joint_];
    z.certificate_available = cert_.available;
    z.certificate_algebra_ok = cert_.algebra_ok;
    z.certificate_initial_ok = cert_initial_ok_;
    z.certificate_monitor_ok = certificate_monitor;
    z.q = r.q[joint_];
    z.dq_physical = r.dq[joint_];
    z.dq_control = velocity;
    z.velocity_bias = b.value;
    z.velocity_bias_derivative = b.derivative;
    z.transformed_d1 = -b.value;
    z.reference = ref.value;
    z.reference_velocity = ref.derivative;
    z.reference_acceleration = ref.second;
    z.lower = low.value;
    z.lower_velocity = low.derivative;
    z.lower_acceleration = low.second;
    z.upper = up.value;
    z.upper_velocity = up.derivative;
    z.upper_acceleration = up.second;
    z.h1 = r.q[joint_] - low.value;
    z.h2 = up.value - r.q[joint_];
    z.psi1_robust = o.psi1;
    z.psi2_robust = o.psi2;
    z.virtual_nominal = o.virtual_nominal;
    z.virtual_target = o.virtual_target;
    z.lambda1 = o.lambda1;
    z.lambda2 = o.lambda2;
    z.k_used = o.k_used;
    z.k_next = o.k_next;
    z.dk_used = o.dk_used;
    z.dhat = o.dhat;
    z.observer_p = o.observer_p;
    z.nominal_acceleration = o.nominal_accel;
    z.barrier_projection = o.barrier_projection;
    z.command_acceleration = o.command_accel;
    z.barrier_lower = o.barrier_lower;
    z.barrier_upper = o.barrier_upper;
    z.feasible_lower = o.feasible_lower;
    z.feasible_upper = o.feasible_upper;
    z.qp_gap = o.qp_gap;
    z.previous_nominal_acceleration = previous_accel_used;
    z.injection_signal = injection_signal;
    z.torque_injected = injection;
    z.configured_injection_amplitude = disturbance_.amplitude;
    z.injection_envelope =
        allow_controlled_injection_ ? injectionEnvelope(t, duration_, disturbance_.ramp_s) : 0;
    if (waveform_ != InjectionWaveform::Invalid)
      z.injection_envelope = shapedInjectionEnvelope(waveform_, t, duration_, disturbance_.ramp_s);
    z.residual_budget_without_injection = residual_bound_;
    z.physical_accel_bound = cfg_.physical_accel_bound;
    z.velocity_error_bound = cfg_.velocity_error_bound;
    z.transformed_accel_bound = transformed_bound_;
    z.mass_diagonal = m;
    z.coriolis = C[joint_];
    z.modeled_friction = friction;
    z.H1 = H1;
    z.H2 = H2;
    z.certificate_R = cert_cfg_.R;
    z.certificate_D_R = cert_.D_R;
    z.control_compute_us = compute_us;
    z.previous_timed_cycle = previous_timed_cycle_;
    z.previous_update_total_us = previous_update_us_;
    for (int j = 0; j < 7; ++j) {
      z.q_all[j] = r.q[j];
      z.dq_all[j] = r.dq[j];
      z.hold_target[j] = hold_[j];
      z.torque_requested[j] = requested[j];
      z.torque_sent[j] = sent[j];
      z.torque_robot_desired[j] = r.tau_J_d[j];
      z.torque_rate_reference[j] = rate_reference[j];
      z.torque_measured[j] = r.tau_J[j];
      z.coriolis_all[j] = C[j];
      z.mass_row[j] = M[7 * j + joint_];
      z.final_clip_changed_all[j] = postclip[j];
    }
    // 仅保存重力项供离线分析；FCI 力矩指令不重复叠加重力。
    const auto G = model_->getGravity();
    for (int j = 0; j < 7; ++j) {
      z.gravity_model[j] = G[j];
      z.start_pose[j] = start_pose_[j];
      z.external_torque_estimate[j] = r.tau_ext_hat_filtered[j];
    }
    for (int j = 0; j < 49; ++j)
      z.mass_matrix_column_major[j] = M[j];
    z.robot_time_s = r.time.toSec();
    z.command_success_rate = r.control_command_success_rate;
    z.configured_tool_mass = r.m_ee;
    z.configured_load_mass = r.m_load;
    z.configured_total_mass = r.m_total;
    for (int j = 0; j < 3; ++j)
      z.configured_total_com[j] = r.F_x_Ctotal[j];
    z.commissioning_guard_enabled = guard_.enabled;
    sample_pub_->unlockAndPublish();
  } else
    ++dropped_;
  previous_update_us_ = elapsedUs(begin);
  previous_timed_cycle_ = cycle_;
}
}  // 命名空间 tase_hcf_rcbf
PLUGINLIB_EXPORT_CLASS(tase_hcf_rcbf::JointComparisonController,
                       controller_interface::ControllerBase)
