#pragma once

#include "lidar_dart_cpp/types.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>

namespace lidar_dart {

class TargetEKF {
 public:
  TargetEKF(const Eigen::Vector3d& initial_position,
            double initial_covariance,
            double process_noise_pos,
            double process_noise_vel,
            const Pose& camera_pose = Pose::Identity())
      : state_(Eigen::VectorXd::Zero(6)),
        cov_(Eigen::MatrixXd::Identity(6, 6) * initial_covariance),
        q_pos_(process_noise_pos),
        q_vel_(process_noise_vel),
        camera_pose_(camera_pose) {
    state_.head<3>() = initial_position;
  }

  void predict(double dt) {
    dt = std::max(dt, 1e-6);
    Eigen::MatrixXd f = Eigen::MatrixXd::Identity(6, 6);
    f.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;

    Eigen::MatrixXd q = Eigen::MatrixXd::Zero(6, 6);
    q.block<3, 3>(0, 0) =
        Eigen::Matrix3d::Identity() * (q_pos_ * dt + q_vel_ * dt * dt * dt / 3.0);
    q.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * (q_vel_ * dt * dt / 2.0);
    q.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity() * (q_vel_ * dt * dt / 2.0);
    q.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * (q_vel_ * dt);

    state_ = f * state_;
    cov_ = f * cov_ * f.transpose() + q;
  }

  void updateLidar(const Eigen::Vector3d& position, double measurement_noise) {
    Eigen::Matrix<double, 3, 6> h_jac = Eigen::Matrix<double, 3, 6>::Zero();
    h_jac.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d r = Eigen::Matrix3d::Identity() * measurement_noise * measurement_noise;
    const Eigen::Vector3d h = state_.head<3>();
    applyUpdate(position, h, h_jac, r);
  }

  void updateCamera(const Eigen::Vector2d& bearing,
                    double measurement_noise_az,
                    double measurement_noise_el) {
    const Pose camera_from_world = camera_pose_.inverse();
    const Eigen::Vector3d p =
        camera_from_world.rotation() * (state_.head<3>() - camera_from_world.translation());

    const double x = p.x();
    const double y = p.y();
    const double z = p.z();
    const double r = std::hypot(x, y);
    const Eigen::Vector2d h(std::atan2(y, x), std::atan2(z, r));

    Eigen::Vector2d residual = bearing - h;
    residual.x() = wrapAngle(residual.x());

    Eigen::Matrix<double, 2, 3> j_bearing = Eigen::Matrix<double, 2, 3>::Zero();
    const double r2 = x * x + y * y;
    if (r2 > 1e-12) {
      j_bearing.row(0) << -y / r2, x / r2, 0.0;
      const double denom = r * (r2 + z * z);
      j_bearing.row(1) << -z * x / denom, -z * y / denom, r / (r2 + z * z);
    }

    Eigen::Matrix<double, 2, 6> h_jac = Eigen::Matrix<double, 2, 6>::Zero();
    h_jac.block<2, 3>(0, 0) = j_bearing * camera_from_world.rotation();
    Eigen::Matrix2d r_noise = Eigen::Matrix2d::Zero();
    r_noise(0, 0) = measurement_noise_az * measurement_noise_az;
    r_noise(1, 1) = measurement_noise_el * measurement_noise_el;
    applyUpdate(bearing, h, h_jac, r_noise, residual);
  }

  Eigen::Vector3d position() const { return state_.head<3>(); }
  Eigen::Vector3d velocity() const { return state_.tail<3>(); }

 private:
  static double wrapAngle(double angle) {
    constexpr double kPi = 3.14159265358979323846;
    while (angle > kPi) angle -= 2.0 * kPi;
    while (angle < -kPi) angle += 2.0 * kPi;
    return angle;
  }

  template <int M>
  void applyUpdate(const Eigen::Matrix<double, M, 1>& z,
                   const Eigen::Matrix<double, M, 1>& h,
                   const Eigen::Matrix<double, M, 6>& h_jac,
                   const Eigen::Matrix<double, M, M>& r,
                   const Eigen::Matrix<double, M, 1>& residual_override) {
    const Eigen::Matrix<double, M, 1> residual = residual_override;
    const Eigen::Matrix<double, M, M> s = h_jac * cov_ * h_jac.transpose() + r;
    const Eigen::Matrix<double, 6, M> gain = cov_ * h_jac.transpose() * s.inverse();
    state_ += gain * residual;
    cov_ = (Eigen::MatrixXd::Identity(6, 6) - gain * h_jac) * cov_;
  }

  void applyUpdate(const Eigen::Vector3d& z,
                   const Eigen::Vector3d& h,
                   const Eigen::Matrix<double, 3, 6>& h_jac,
                   const Eigen::Matrix3d& r) {
    applyUpdate<3>(z, h, h_jac, r, z - h);
  }

  Eigen::VectorXd state_;
  Eigen::MatrixXd cov_;
  double q_pos_;
  double q_vel_;
  Pose camera_pose_;
};

}  // namespace lidar_dart
