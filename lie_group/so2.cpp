#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include "so2.h"


double SO2::unhat(Eigen::Matrix2d theta_hat) {

  return theta_hat(1,0);
}


Eigen::Matrix2d SO2::skewSymmetric(double theta) {
  Eigen::Matrix2d theta_hat;
  theta_hat.setZero();

  theta_hat(0,1) = -theta;
  theta_hat(1,0) = theta;

  return theta_hat;
}


Eigen::Matrix2d SO2::ExpMap(double theta) {
  Eigen::Matrix2d rot_2d;
  rot_2d.setZero();

  rot_2d(0,0) = std::cos(theta);
  rot_2d(1,1) = std::cos(theta);
  rot_2d(0,1) = -std::sin(theta);
  rot_2d(1,0) = std::sin(theta);

  return rot_2d;
}


double SO2::LogMap(Eigen::Matrix2d R) {

  return std::atan2(R(1,0), R(0,0));
}


double SO2::ProductLogMap(Eigen::Matrix2d R1, Eigen::Matrix2d R2) {

  return SO2::LogMap(R1) + SO2::LogMap(R2);
}
