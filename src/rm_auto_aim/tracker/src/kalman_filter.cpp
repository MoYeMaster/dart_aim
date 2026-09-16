#include "tracker/kalman_filter.hpp"

namespace rm_auto_aim
{
KalmanFilter::KalmanFilter()
{
    //init
    x_ = Eigen::VectorXd(4);
    x_.setZero();
    P_ = Eigen::MatrixXd(4,4);
    P_.setIdentity();
    P_ *= 100;

    Q_ = Eigen::MatrixXd::Identity(4, 4) * 0.1;
    R_ = Eigen::MatrixXd::Identity(2, 2) * 0.5;

    H_ = Eigen::MatrixXd::Zero(2,4);
    H_(0, 0) = 1.0; 
    H_(1, 1) = 1.0; 
}


void KalmanFilter::predict(double dt)
{
    F_ = Eigen::MatrixXd::Identity(4, 4);
    F_(0,2) = dt;
    F_(1,3) = dt;

    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

Eigen::VectorXd KalmanFilter::update(double zx, double zy)
{
    Eigen::VectorXd z(2);
    z << zx, zy;

    Eigen::MatrixXd Ht = H_.transpose();
    Eigen::MatrixXd S = H_ * P_ * Ht + R_;
    Eigen::MatrixXd K = P_ * Ht * S.inverse();

    x_ = x_ + K * (z - H_ * x_);
    P_ = (I_ - K * H_) * P_;
    return x_;
}

void KalmanFilter::setProcessNoise(double qp,double qv)
{
    Q_ << qp,0,0,0,
          0,qp,0,0,
          0,0,qv,0,
          0,0,0,qv;
}

void KalmanFilter::setMeasurementNoise(double r)
{
    R_ << r,0,
          0,r;
}
}