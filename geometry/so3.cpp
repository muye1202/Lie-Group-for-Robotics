#include "so3.h"
#include <cmath>
#include <Eigen/Dense>


double SO3::SO3toAngle(Eigen::Matrix3d mat) {
  
  double rot_angle = std::acos((mat.trace() - 1) / 2.0);

  return rot_angle;
}


Eigen::Vector3d SO3::unhat(const Eigen::Matrix3d& matrix) {
  Eigen::Vector3d vec;
  vec << matrix(2,1), matrix(0, 2), matrix(1, 0);

  return vec;
}


Eigen::Matrix3d SO3::skewSymmetric(const Eigen::Vector3d& vec) {
  Eigen::Matrix3d skew;
  skew <<  0,      -vec.z(),  vec.y(),
          vec.z(),  0,      -vec.x(),
          -vec.y(),  vec.x(),  0;
  return skew;
}


Eigen::Matrix3d SO3::ExpMap(Eigen::Vector3d w) {
  // get the rotation angle
  double rot_angle = w.norm();
  
  // get the unit rot vector
  Eigen::Vector3d unit_rot = w.normalized();

  // exponential map: so(3) -> SO(3)
  Eigen::Matrix3d exp_map = std::cos(rot_angle) * Eigen::MatrixXd::Identity(3,3)
                            + (1 - std::cos(rot_angle)) * unit_rot * unit_rot.transpose()
                            + std::sin(rot_angle) * skewSymmetric(unit_rot);

  return exp_map;
}


Eigen::Vector3d SO3::LogMap(Eigen::Matrix3d R) {
  double rotation_angle = SO3::SO3toAngle(R);

  double sin_theta = 0.5 * std::sqrt((3 - R.trace()) * (1 + R.trace()));

  Eigen::Matrix3d R_RT = R - R.transpose();
  Eigen::Vector3d axis_rot = (rotation_angle * SO3::unhat(R_RT)) / (2 * sin_theta);

  return axis_rot;
}


Eigen::Matrix3d SO3::leftJacobian(Eigen::Vector3d w) {
  // get the rotation angle
  double rot_angle = w.norm();
  
  // get the unit rot vector
  Eigen::Vector3d unit_rot = w.normalized();

  Eigen::Matrix3d left_J = (std::sin(rot_angle) / rot_angle) * Eigen::MatrixXd::Identity(3,3) + 
                           (1 - std::sin(rot_angle) / rot_angle) * unit_rot * unit_rot.transpose() + 
                           skewSymmetric(unit_rot) * (1 - std::cos(rot_angle)) / rot_angle;

  return left_J;
}


Eigen::Matrix3d SO3::leftJacobian_inv(Eigen::Vector3d w) {
  // get the rotation angle
  double rot_angle = w.norm();
  
  // get the unit rot vector
  Eigen::Vector3d unit_rot = w.normalized();

  Eigen::Matrix3d leftJ_inv = (0.5*rot_angle) * 1./(std::tan(0.5*rot_angle)) * Eigen::MatrixXd::Identity(3,3) +
                              (1 - (0.5*rot_angle) * 1./(std::tan(0.5*rot_angle))) * unit_rot * unit_rot.transpose() -
                              (0.5*rot_angle) * skewSymmetric(unit_rot);

  return leftJ_inv;
}


Eigen::Matrix3d SO3::rightJacobian(Eigen::Vector3d w) {
  // get the rotation angle
  double rot_angle = w.norm();
  
  // get the unit rot vector
  Eigen::Vector3d unit_rot = w.normalized();

  Eigen::Matrix3d Jr = (std::sin(rot_angle) / rot_angle) * Eigen::MatrixXd::Identity(3,3) + 
                           (1 - std::sin(rot_angle) / rot_angle) * unit_rot * unit_rot.transpose() - 
                           skewSymmetric(unit_rot) * (1 - std::cos(rot_angle)) / rot_angle;

  return Jr;
}


Eigen::Matrix3d SO3::rightJacobian_inv(Eigen::Vector3d w) {
  // get the rotation angle
  double rot_angle = w.norm();
  
  // get the unit rot vector
  Eigen::Vector3d unit_rot = w.normalized();

  Eigen::Matrix3d Jr_inv = (0.5*rot_angle) * 1./(std::tan(0.5*rot_angle)) * Eigen::MatrixXd::Identity(3,3) +
                              (1 - (0.5*rot_angle) * 1./(std::tan(0.5*rot_angle))) * unit_rot * unit_rot.transpose() +
                              (0.5*rot_angle) * skewSymmetric(unit_rot);

  return Jr_inv;
}


Eigen::Matrix3d SO3::J_JT(Eigen::Vector3d w) {
  // get the rotation angle
  double rot_angle = w.norm();

  double gamma = 2*(1-std::cos(rot_angle)) / (std::pow(rot_angle,2));

  // get the unit rot vector
  Eigen::Vector3d unit_rot = w.normalized();

  Eigen::Matrix3d JJT = gamma * Eigen::MatrixXd::Identity(3,3) +
                        (1 - gamma) * unit_rot * unit_rot.transpose();

  return JJT;
}


Eigen::Matrix3d SO3::J_JT_inv(Eigen::Vector3d w) {
  // get the rotation angle
  double rot_angle = w.norm();

  double gamma = 2*(1-std::cos(rot_angle)) / (std::pow(rot_angle,2));

  // get the unit rot vector
  Eigen::Vector3d unit_rot = w.normalized();

  Eigen::Matrix3d JJT_inv = (1./gamma) * Eigen::MatrixXd::Identity(3,3) +
                        (1 - 1./gamma) * unit_rot * unit_rot.transpose();

  return JJT_inv;
}


SO3::Rotation::Rotation(Eigen::Vector3d w) : rot(w) {
  Eigen::Matrix3d exp_map = SO3::ExpMap(w);
  mat_rot = exp_map;
}


SO3::Rotation::Rotation(Eigen::Matrix3d rotation_mat) : mat_rot(rotation_mat) {
  rot = SO3::LogMap(rotation_mat);
} 


Eigen::Matrix3d SO3::Rotation::GetMatrix() {

  return mat_rot;
}


double SO3::Rotation::get(int row, int col) {

  return mat_rot(row, col);
}


SO3::Rotation SO3::Rotation::transpose() {
  Eigen::Matrix3d rt = this->mat_rot.transpose();
  SO3::Rotation RT(rt);

  return RT;
}


SO3::Rotation SO3::Rotation::inverse() {
  return this->transpose();
}
