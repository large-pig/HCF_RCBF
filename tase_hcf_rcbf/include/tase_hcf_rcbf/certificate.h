/**
 * @file certificate.h
 * @brief HCF 障碍函数的代数充分条件检查。
 *
 * 检查使用配置给定的扰动界和跟踪包络。
 * 代数条件成立不代表真实机器人所受扰动已经得到实验辨识或独立验证。
 */

#pragma once

#include "signals.h"

namespace tase_hcf_rcbf {
// 这里只检查 n=2 条件下的代数充分条件。L1、mu1/mu2、P2
// 必须由外部分析提供依据，本函数不会辨识实际物理扰动界。
struct CertificateConfig {
  double R = -1;
  double nu = .01;
  double L1 = -1;
  double K1 = -1;
  double P2 = -1;
  double mu1 = -1;
  double mu2 = -1;
};
struct Certificate {
  bool available = false;
  bool algebra_ok = false;
  double r1 = 0;
  double r2 = 0;
  double Psi = 0;
  double ell = 0;
  double D_R = 0;
  double sharing_margin = 0;
};
// 使用保守的全局信号包络检查 HCF 的充分条件。
inline Certificate checkCertificate(const Config& c, const CertificateConfig& a,
                                    const Harmonic& lower, const Harmonic& upper) {
  Certificate v;
  const double vals[] = {a.R, a.nu, a.L1, a.K1, a.P2, a.mu1, a.mu2};
  for (double x : vals)
    if (!finite(x))
      return v;
  if (a.R <= 0 || a.nu <= 0 || a.L1 < 0 || a.K1 < 0 || a.P2 < 0 || a.mu1 < 0 || a.mu2 < 0 ||
      a.P2 < a.mu2 || !validConfig(c) || !validHarmonic(lower) || !validHarmonic(upper))
    return v;
  v.r1 = a.R * a.R * c.p1 / (4 * c.eta1);
  v.r2 = a.R * a.R * c.p2 / (4 * c.eta1);
  const double lmin = lower.offset - std::abs(lower.amplitude),
               umax = upper.offset + std::abs(upper.amplitude);
  const double xmin = lmin - v.r1 - a.nu, xmax = umax + v.r2 + a.nu;
  v.Psi = std::max(1 + v.r1 * std::exp(xmax - lmin), 1 + v.r2 * std::exp(umax - xmin));
  // 独立包络界较为保守，但适用于不同信号相位组合。
  const double sigma =
      upper.offset - std::abs(upper.amplitude) - lower.offset - std::abs(lower.amplitude);
  const double vy =
      std::abs(upper.amplitude * upper.omega) + std::abs(lower.amplitude * lower.omega);
  v.sharing_margin =
      c.eta1 * sigma - vy - std::exp(v.r1 + a.nu) / c.p1 - std::exp(v.r2 + a.nu) / c.p2;
  v.ell = c.eta2 - c.eta1 * v.Psi / 2;
  if (v.ell > 0)
    v.D_R = a.mu1 + c.tau * a.L1 + std::pow(v.Psi + 1 + a.mu2 + a.P2, 2) / (4 * v.ell);
  else
    v.D_R = std::numeric_limits<double>::infinity();
  v.available = finite(v.Psi) && finite(v.sharing_margin) && finite(v.D_R);
  v.algebra_ok = v.available && a.R > a.mu1 && v.ell > 0 && v.sharing_margin >= 0 && v.D_R <= a.R;
  return v;
}
// 从原始安全函数中扣除虚拟速度跟踪误差项，构造复合障碍函数。
inline double compositeH(double h, double r, double velocity, double k) {
  return h + r * std::exp(h) - .5 * (velocity - k) * (velocity - k);
}
}  // 命名空间 tase_hcf_rcbf
