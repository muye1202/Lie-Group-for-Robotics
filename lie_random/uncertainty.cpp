#include <Eigen/Dense>
#include "se3.h"
#include "so3.h"
#include "uncertainty.h"


SO3::Rotation LIE_RANDOM::left_random_R(SO3::Rotation R_mean, Eigen::Vector3d eps) {

  Eigen::Matrix3d R_left = SO3::ExpMap(eps) * R_mean.GetMatrix();

  return SO3::Rotation(R_left);
}


SO3::Rotation LIE_RANDOM::update_R_mean(SO3::Rotation R, SO3::Rotation R_mean) {

  return R*R_mean;
}


Eigen::Matrix3d LIE_RANDOM::update_R_covariance(SO3::Rotation R, Eigen::Matrix3d cov_matrix) {

  return R.GetMatrix() * cov_matrix * R.GetMatrix().transpose();
}


SE3::Pose LIE_RANDOM::update_T_mean(SE3::Pose update, SE3::Pose old_mean) {

  return update * old_mean;
}


Eigen::MatrixXd LIE_RANDOM::update_T_covariance(SE3::Pose update, Eigen::MatrixXd old_cov) {

  Eigen::MatrixXd Ad_T = update.adjoint();

  return Ad_T * old_cov * Ad_T.transpose();
}
