/**
 * @file core.h
 * @brief 独立于 ROS 和硬件接口的 HCF / RCBF 控制算法。
 *
 * 核心模块输入与输出的加速度单位均为 rad/s²，并接收执行器允许的加速度区间。
 * 本文件不直接向机器人下发命令；动力学换算和最终力矩限制由 ROS 控制器完成。
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

namespace tase_hcf_rcbf {
inline double clamp(double x, double lo, double hi) {
  return std::max(lo, std::min(hi, x));
}
inline bool finite(double x) {
  return std::isfinite(x);
}
inline bool different(double a, double b) {
  return std::abs(a - b) > 1e-9 * (1 + std::max(std::abs(a), std::abs(b)));
}
// 两种算法共享相同的输入输出结构及执行器可行域定义。

enum class Mode { HCF = 0, RCBF = 1 };
// 以下是控制参数的初始化默认值。最终实验由 HCF / RCBF 的 YAML 参数
// 覆盖，不能将这些默认值直接视为实机最终参数。
struct Config {
  Mode mode = Mode::HCF;

  // HCF 虚拟控制、误差反馈和扰动观测器参数。
  double ke = 1;
  double eta1 = 5;
  double eta2 = 100;
  double p1 = 1;
  double p2 = 1;
  double tau = 0.001;
  double rho = 0.1;
  double observer_gain = 10;

  // RCBF 名义跟踪增益及二阶安全障碍参数。
  double kp = 100;
  double kd = 20;
  double k1 = 10;
  double k2 = 10;
  double physical_accel_bound = -1;
  double velocity_error_bound = 0;
};

// 控制输入：实际状态、期望轨迹以及动态安全边界的导数。
// 所有加速度量统一采用 rad/s²。
struct Input {
  double q = 0;
  double velocity = 0;
  double qd = 0;
  double dqd = 0;
  double ddqd = 0;

  double lower = 0;
  double dlower = 0;
  double ddlower = 0;
  double upper = 0;
  double dupper = 0;
  double ddupper = 0;

  double dt = 0;
  double previous_nominal_accel = 0;
  double accel_min = -std::numeric_limits<double>::infinity();
  double accel_max = std::numeric_limits<double>::infinity();
};

// HCF 滤波器与扰动观测器的跨周期内部状态。
// 仅当本周期控制结果有效时更新内部状态。
struct State {
  bool initialized = false;
  double k = 0;
  double dhat = 0;
  double previous_velocity = 0;
};

// 控制结果及实验分析字段，后续由 TrialSample 消息发布。
// command_accel 为满足约束的加速度指令，实际力矩在 ROS 控制层换算。
struct Output {
  bool valid = false;
  bool qp_feasible = false;
  bool initial_safe = false;
  bool top_filter_active = false;
  bool safety_filter_active = false;
  bool nominal_outside_actuator = false;
  bool actuator_projection_changed = false;
  bool actuator_constraint_active = false;

  double h1 = 0;
  double h2 = 0;
  double psi1 = 0;
  double psi2 = 0;
  double virtual_nominal = 0;
  double virtual_target = 0;
  double lambda1 = 0;
  double lambda2 = 0;
  double k_used = 0;
  double k_next = 0;
  double dk_used = 0;
  double dhat = 0;
  double observer_p = 0;
  double nominal_accel = 0;
  double barrier_projection = 0;
  double raw_accel = 0;
  double command_accel = 0;
  double barrier_lower = 0;
  double barrier_upper = 0;
  double feasible_lower = 0;
  double feasible_upper = 0;
  double qp_gap = 0;
};
// 检查算法增益和扰动界配置，拒绝不满足前提的参数。
inline bool validConfig(const Config& c) {
  const double v[] = {c.ke,
                      c.eta1,
                      c.eta2,
                      c.p1,
                      c.p2,
                      c.tau,
                      c.rho,
                      c.observer_gain,
                      c.kp,
                      c.kd,
                      c.k1,
                      c.k2,
                      c.physical_accel_bound,
                      c.velocity_error_bound};
  for (double a : v)
    if (!finite(a))
      return false;
  if (c.velocity_error_bound < 0)
    return false;
  if (c.mode == Mode::HCF)
    return c.ke >= 0 && c.eta1 > 0 && c.eta2 > 0 && c.p1 > 0 && c.p2 > 0 && c.tau > 0 &&
           c.rho > 0 && c.observer_gain > 0;
  return c.mode == Mode::RCBF && c.kp >= 0 && c.kd >= 0 && c.k1 > 0 && c.k2 > 0 &&
         c.physical_accel_bound >= 0;
}
/**
 * @brief 在不依赖 ROS 的情况下计算一周期 HCF / RCBF 控制指令。
 *
 * 对于 HCF：先将名义虚拟速度投影到一阶安全障碍允许区间，
 * 再进行滤波和扰动观测补偿，得到实际跟踪加速度。
 * 对于 RCBF：将名义加速度投影至二阶障碍条件与执行器约束的
 * 交集区间，避免在控制器输出中生成不可执行的指令。
 * 如果可行域为空或存在非有限数值，返回无效结果，
 * 并保持上一周期的滤波器状态不变。
 */
inline Output step(const Config& c, const Input& i, State& s) {
  Output o;
  if (!validConfig(c))
    return o;
  const double values[] = {i.q,
                           i.velocity,
                           i.qd,
                           i.dqd,
                           i.ddqd,
                           i.lower,
                           i.dlower,
                           i.ddlower,
                           i.upper,
                           i.dupper,
                           i.ddupper,
                           i.dt,
                           i.previous_nominal_accel};
  for (double a : values)
    if (!finite(a))
      return o;
  if (i.dt <= 0 || i.lower >= i.upper || std::isnan(i.accel_min) || std::isnan(i.accel_max) ||
      i.accel_min > i.accel_max)
    return o;
  // 计算关节位置到上下安全边界的有符号裕度。
  // 两个裕度均非负表示关节处于原始安全区间内。
  o.h1 = i.q - i.lower;
  o.h2 = i.upper - i.q;
  State next = s;
  o.initial_safe = o.h1 >= 0 && o.h2 >= 0;
  if (c.mode == Mode::HCF) {
    // 第一层：依据上下边界的一阶障碍约束，
    // 把名义虚拟速度投影到可行的速度区间。
    const double a1 = -i.dlower - std::exp(-o.h1) / c.p1 + c.eta1 * o.h1;
    const double a2 = i.dupper - std::exp(-o.h2) / c.p2 + c.eta1 * o.h2;
    o.virtual_nominal = i.dqd - c.ke * (i.q - i.qd);
    if (!finite(a1) || !finite(a2) || !finite(o.virtual_nominal))
      return o;
    o.qp_gap = std::max(0.0, -a1 - a2);
    o.qp_feasible = o.qp_gap == 0;
    if (!o.qp_feasible)
      return o;
    o.lambda1 = std::max(0.0, -a1 - o.virtual_nominal);
    o.lambda2 = std::max(0.0, o.virtual_nominal - a2);
    o.virtual_target = o.virtual_nominal + o.lambda1 - o.lambda2;
    o.top_filter_active = different(o.virtual_nominal, o.virtual_target);
    // 扰动观测器使用上一周期名义加速度、
    // 当前速度增量与采样周期更新估计量。
    if (!next.initialized) {
      next.initialized = true;
      next.k = o.virtual_target;
      next.dhat = 0;
      next.previous_velocity = i.velocity;
    } else {
      // 假设采样区间内加速度常值时使用离散更新表达式；
      // 不将其推广为任意连续运动的精确离散等价。
      const double weight = -std::expm1(-c.observer_gain * i.dt);
      next.dhat += weight * ((i.velocity - next.previous_velocity) / i.dt -
                             i.previous_nominal_accel - next.dhat);
    }
    // 第二层：跟踪滤波后的安全虚拟速度，
    // 并补偿观测器估计的加速度扰动。
    o.k_used = next.k;
    o.dk_used = (o.virtual_target - o.k_used) / c.tau;
    o.dhat = next.dhat;
    o.observer_p = o.dhat - c.observer_gain * i.velocity;
    o.raw_accel = std::tanh((o.lambda1 - o.lambda2) / c.rho) + o.dk_used - o.dhat -
                  c.eta2 * (i.velocity - o.k_used);
    o.nominal_accel = o.raw_accel;
    o.barrier_projection = o.raw_accel;
    o.feasible_lower = i.accel_min;
    o.feasible_upper = i.accel_max;
    o.command_accel = clamp(o.raw_accel, i.accel_min, i.accel_max);
    next.k += -std::expm1(-i.dt / c.tau) * (o.virtual_target - next.k);
    o.k_next = next.k;
    next.previous_velocity = i.velocity;
  } else {
    // RCBF 使用二阶模型 q̈ = a + d。
    // 若测量速度含有有界偏差 b，安全裕度应同时考虑
    // 已配置的加速度扰动预算及由速度偏差引入的附加影响
    // (k1+k2)|b|，这里不需要使用偏差的一阶导数。
    const double ks = c.k1 + c.k2, kp = c.k1 * c.k2;
    const double margin = c.physical_accel_bound + ks * c.velocity_error_bound;
    o.psi1 = i.velocity - i.dlower + c.k1 * o.h1 - c.velocity_error_bound;
    o.psi2 = i.dupper - i.velocity + c.k1 * o.h2 - c.velocity_error_bound;
    o.initial_safe = o.initial_safe && o.psi1 >= 0 && o.psi2 >= 0;
    o.barrier_lower = i.ddlower - ks * (i.velocity - i.dlower) - kp * o.h1 + margin;
    o.barrier_upper = i.ddupper - ks * (i.velocity - i.dupper) + kp * o.h2 - margin;
    if (!finite(o.barrier_lower) || !finite(o.barrier_upper))
      return o;
    // 安全指令同时满足障碍约束和执行器幅值/变化率限制。
    // 若二者交集为空，本周期标记为不可行。
    o.feasible_lower = std::max(i.accel_min, o.barrier_lower);
    o.feasible_upper = std::min(i.accel_max, o.barrier_upper);
    o.qp_gap = std::max(0.0, o.feasible_lower - o.feasible_upper);
    o.qp_feasible = o.qp_gap == 0;
    if (!o.qp_feasible)
      return o;
    o.nominal_accel = i.ddqd - c.kd * (i.velocity - i.dqd) - c.kp * (i.q - i.qd);
    o.raw_accel = o.nominal_accel;
    o.barrier_projection = clamp(o.nominal_accel, o.barrier_lower, o.barrier_upper);
    o.safety_filter_active = different(o.nominal_accel, o.barrier_projection);
    o.command_accel = clamp(o.nominal_accel, o.feasible_lower, o.feasible_upper);
  }
  o.nominal_outside_actuator = o.raw_accel < i.accel_min || o.raw_accel > i.accel_max;
  o.actuator_projection_changed = different(o.barrier_projection, o.command_accel);
  o.actuator_constraint_active =
      (finite(i.accel_min) && !different(o.command_accel, i.accel_min)) ||
      (finite(i.accel_max) && !different(o.command_accel, i.accel_max));
  const double outputs[] = {o.command_accel, o.raw_accel, o.dhat, o.k_next, o.dk_used};
  for (double a : outputs)
    if (!finite(a))
      return o;
  // 仅在全部有效性检查通过后提交跨周期状态。
  s = next;
  o.valid = true;
  return o;
}
}  // 命名空间 tase_hcf_rcbf
