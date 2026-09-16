#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

namespace rm_auto_aim
{
class KalmanFilter
{
public:
  KalmanFilter();
  
  void init(const Eigen::VectorXd& initial_state,const Eigen::MatrixXd& convariance);
  void predict(double dt);
  Eigen::VectorXd update(double zx, double zy);
  Eigen::VectorXd getState() const{ return x_;  }
  Eigen::MatrixXd getConvariance() const {  return P_;  }
  void setProcessNoise(double qp,double qv);
  void setMeasurementNoise(double r);

private:
  Eigen::VectorXd x_;  // 状态向量
  Eigen::MatrixXd P_;  // 协方差矩阵
  Eigen::MatrixXd F_;  // 状态转移
  Eigen::MatrixXd H_;  // 观测矩阵
  Eigen::MatrixXd Q_;  // 过程噪声
  Eigen::MatrixXd R_;  // 观测噪声
  Eigen::MatrixXd I_;  // 单位矩阵
};
}  // namespace rm_auto_aim

#endif  // KALMAN_FILTER_HPP