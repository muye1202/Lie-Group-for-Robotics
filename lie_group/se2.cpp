#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include "se2.h"


SE2::Pose::Pose() {
  _pose = Eigen::Matrix3d::Identity(3,3);
  _R = Eigen::Matrix2d::Identity(2,2);
  _t = Eigen::Vector2d::Zero();
  _linear = Eigen::Vector2d::Zero();
  _angular = 0.0;
}


SE2::Pose::Pose(Eigen::Vector3d twist) {
  _pose = SE2::ExpMap(twist);
  _R = _pose.block(0,0,2,2);
  _t = _pose.block(0,2,2,1);
  _linear = twist.segment(0,2);
  _angular = twist(2);
}


SE2::Pose::Pose(Eigen::Matrix3d T) {
  _pose = T;
  _R = T.block(0,0,2,2);
  _t = T.block(0,2,2,1);

  Eigen::Vector3d tau = SE2::LogMap(_pose);
  _linear = tau.segment(0,2);
  _angular = tau(2);
}


SE2::Pose::Pose(Eigen::Matrix2d R, Eigen::Vector2d t) {

  Eigen::Matrix3d T = Eigen::Matrix3d::Identity(3,3);
  T.block(0,0,2,2) = R;
  T.block(0,2,2,1) = t;
  _pose = T;

  _R = R;
  _t = t;

  Eigen::Vector3d tau = SE2::LogMap(_pose);
  _linear = tau.segment(0,2);
  _angular = tau(2);
}


SE2::Pose::Pose(double theta, Eigen::Vector2d t) {

  _R = SO2::ExpMap(theta);
  _t = t;

  Eigen::Matrix3d T = Eigen::Matrix3d::Identity(3,3);
  T.block(0,0,2,2) = _R;
  T.block(0,2,2,1) = _t;
  _pose = T;

  Eigen::Vector3d tau = SE2::LogMap(_pose);
  _linear = tau.segment(0,2);
  _angular = tau(2);
}


Eigen::Matrix3d SE2::Pose::inverse() {

  Eigen::Matrix3d T = Eigen::Matrix3d::Identity(3,3);
  T.block(0,0,2,2) = _R.transpose();
  T.block(0,2,2,1) = -1*_R.transpose() * _t;

  return T;
}


SE2::Pose SE2::Pose::operator*(Pose& other) {

  Eigen::Matrix3d T = Eigen::Matrix3d::Identity(3,3);
  T.block(0,0,2,2) = _R * other.GetRotation();
  T.block(0,2,2,1) = _t + _R * other.GetTranslation();

  return SE2::Pose(T);
}


Eigen::Matrix3d SE2::skewSymmetric(double angular_vel, Eigen::Vector2d linear_vel) {

  Eigen::Matrix3d tau_hat = Eigen::Matrix3d::Zero();

  tau_hat.block(0,0,2,2) = SO2::skewSymmetric(angular_vel);
  tau_hat.block(0,2,2,1) = linear_vel;

  return tau_hat;
}


Eigen::Vector3d SE2::unhat(Eigen::Matrix3d tau_hat) {

  Eigen::Vector3d result = Eigen::Vector3d::Zero();

  Eigen::Matrix2d theta_hat = tau_hat.block(0,0,2,2);
  result(2,0) = SO2::unhat(theta_hat);

  result.segment(0,2) = tau_hat.block(0,2,2,1);

  return result;
}


Eigen::Matrix3d SE2::ExpMap(Eigen::Vector3d tau) {

  Eigen::Matrix3d T = Eigen::Matrix3d::Zero();
  T(2,2) = 1.0;

  Eigen::Vector2d linear_vel = tau.segment(0,2);
  double angular_vel = tau(2,0);

  Eigen::Matrix2d V = (std::sin(angular_vel)/angular_vel) * Eigen::Matrix2d::Identity(2,2) +
                      SO2::skewSymmetric(1.0) * (1-std::cos(angular_vel)) / angular_vel;

  T.block(0,0,2,2) = SO2::ExpMap(angular_vel);
  T.block(0,2,2,1) = V*linear_vel;

  return T;
}


Eigen::Vector3d SE2::LogMap(Eigen::Matrix3d T) {

  Eigen::Vector2d translation = T.block(0,2,2,1);
  Eigen::Matrix2d rotation = T.block(0,0,2,2);

  double theta = SO2::unhat(rotation);
  Eigen::Matrix2d V = (std::sin(theta)/theta) * Eigen::Matrix2d::Identity(2,2) +
                      SO2::skewSymmetric(1.0) * (1-std::cos(theta)) / theta;

  Eigen::Vector2d linear = V.inverse() * translation;
  double angular = SO2::LogMap(rotation);

  Eigen::Vector3d result;
  result << linear, angular;

  return result;
}
