#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include "se2.h"


Eigen::Matrix3d SE2::skewSymmetric(double angular_pos, Eigen::Vector2d linear_pos) {

  Eigen::Matrix3d tau_hat = Eigen::Matrix3d::Zero();

  tau_hat.block(0,0,2,2) = SO2::skewSymmetric(angular_pos);
  tau_hat.block(0,2,2,1) = linear_pos;

  return tau_hat;
}


Eigen::Vector3d SE2::unhat(Eigen::Matrix3d tau_hat) {

  Eigen::Vector3d result = Eigen::Vector3d::Zero();

  Eigen::Matrix2d theta_hat = tau_hat.block(0,0,2,2);
  result(2,0) = SO2::unhat(theta_hat);

  result.segment(0,2) = tau_hat.block(0,2,2,1);

  return result;
}


// Eigen::Matrix3d SE2::ExpMap(Eigen::Vector3d tau) {

// }
