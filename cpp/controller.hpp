// C++ port of src/impedance.py (Cartesian impedance + nullspace) and m1_press.py's ForcePI.
//
//   tau = J^T (K e - D xdot) + c(q, qdot) + N^T tau_null
//
// No allocation: every matrix has a fixed size, so nothing touches the heap inside the 1 kHz loop.
#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <mujoco/mujoco.h>

namespace ffc {

using Vec3 = Eigen::Vector3d;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Vec7 = Eigen::Matrix<double, 7, 1>;
using Mat6 = Eigen::Matrix<double, 6, 6>;
using Mat7 = Eigen::Matrix<double, 7, 7>;
using Mat67 = Eigen::Matrix<double, 6, 7>;
using Mat76 = Eigen::Matrix<double, 7, 6>;

struct Gains {
  double k_pos = 500.0;
  double k_rot = 50.0;
  double k_null = 10.0;
};

struct Inputs {
  Vec3 pos;
  double quat[4];
  Mat67 J;
  Mat7 M;
  Vec7 q, qd, bias;
  Vec3 pos_des;
  double quat_des[4];
  bool has_override_z;
  double override_z;
};

// Same cutoff as numpy.linalg.pinv's default rcond = 1e-15.
inline Mat6 pinv(const Mat6& A) {
  Eigen::JacobiSVD<Mat6> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const Vec6 s = svd.singularValues();
  const double cutoff = 1e-15 * s(0);
  Vec6 si;
  for (int i = 0; i < 6; ++i) si(i) = s(i) > cutoff ? 1.0 / s(i) : 0.0;
  return svd.matrixV() * si.asDiagonal() * svd.matrixU().transpose();
}

inline Vec7 impedance_torque(const Inputs& in, const Vec7& q_rest, const Gains& g) {
  double conj[4], err[4], rotvec[3];
  mju_negQuat(conj, in.quat);
  mju_mulQuat(err, in.quat_des, conj);
  mju_quat2Vel(rotvec, err, 1.0);

  Vec6 spring;
  spring.head<3>() = g.k_pos * (in.pos_des - in.pos);
  spring.tail<3>() = g.k_rot * Eigen::Map<const Vec3>(rotvec);
  if (in.has_override_z) spring(2) = in.override_z;

  Vec6 d;
  d.head<3>().setConstant(2.0 * std::sqrt(g.k_pos));
  d.tail<3>().setConstant(2.0 * std::sqrt(g.k_rot));
  const Vec6 wrench = spring - d.cwiseProduct(in.J * in.qd);
  const Vec7 tau_task = in.J.transpose() * wrench;

  // Dynamically consistent nullspace projector N = I - J^T Jbar^T.
  const Mat7 Minv = in.M.inverse();
  const Mat76 MinvJt = Minv * in.J.transpose();
  const Mat6 Lambda = pinv(in.J * MinvJt);
  const Mat76 Jbar = MinvJt * Lambda;
  const Mat7 N = Mat7::Identity() - in.J.transpose() * Jbar.transpose();
  const double d_null = 2.0 * std::sqrt(g.k_null);
  const Vec7 tau_null = N * (g.k_null * (q_rest - in.q) - d_null * in.qd);

  return tau_task + tau_null + in.bias;
}

struct ForcePI {
  double kp = 0.5, ki = 20.0, ramp = 0.02, i_max = 10.0, f_des = 10.0;
  double integral = 0.0, f0 = 0.0, t0 = 0.0;

  void reset(double f_switch, double t_switch) {
    integral = 0.0;
    f0 = f_switch;
    t0 = t_switch;
  }
  double reference(double t) const {
    const double a = std::min(1.0, (t - t0) / ramp);
    return f0 + a * (f_des - f0);
  }
  double wrench_z(double f_meas, double t, double dt) {
    const double f_ref = reference(t);
    const double e = f_ref - f_meas;
    integral = std::clamp(integral + ki * e * dt, -i_max, i_max);
    return -(f_ref + kp * e + integral);
  }
};

}  // namespace ffc
