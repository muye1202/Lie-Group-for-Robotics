#include "se3.h"
#include <Eigen/Dense>


Eigen::Matrix4d SE3::skewSymmetric(const Eigen::VectorXd& eksi) {
  Eigen::Vector3d pos = eksi.block(3,0,3,1);
  Eigen::Vector3d rot_vec = eksi.block(0,0,3,1);

  Eigen::Matrix3d phi_hat = SO3::skewSymmetric(rot_vec);

  Eigen::Matrix4d pos_hat;
  pos_hat.setZero();
  pos_hat.block(0,0,3,3) = phi_hat;
  pos_hat.block(0,3,3,1) = rot_vec;

  return pos_hat;
}


Eigen::Matrix4d SE3::ExpMap(Eigen::VectorXd eksi) {
  // get the rotation angle
  Eigen::Vector3d rot_vec = eksi.block(0,0,3,1);
  double phi = rot_vec.norm();

  // pose vector hat matrix
  Eigen::Matrix4d eksi_hat = SE3::skewSymmetric(eksi);

  Eigen::Matrix4d T = Eigen::MatrixXd::Identity(4,4) + eksi_hat + 
                      ((1-std::cos(phi))/std::pow(phi, 2)) * eksi_hat * eksi_hat +
                      ((phi - std::sin(phi))/std::pow(phi, 3)) * eksi_hat * eksi_hat * eksi_hat;

  return T;
}


SE3::Pose::Pose(Eigen::Vector3d rot, Eigen::Vector3d trans) {
  rotation = SO3::Rotation(rot);
  translation = trans;

  Eigen::Matrix4d T;
  T.block(0,0,3,3) = rotation.GetMatrix();
  T.block(0,3,3,1) = trans;

  T(3,3) = 1.0;

  this->pose = T;
}


SE3::Pose::Pose(Eigen::Matrix3d R, Eigen::Vector3d trans) {
  rotation = SO3::Rotation(R);
  translation = trans;

  Eigen::Matrix4d T;
  T.block(0,0,3,3) = R;
  T.block(0,3,3,1) = trans;

  T(3,3) = 1.0;

  this->pose = T;
}


SE3::Pose::Pose(SO3::Rotation R, Eigen::Vector3d trans) {
  rotation = R;
  translation = trans;

  Eigen::Matrix4d T;
  T.block(0,0,3,3) = R.GetMatrix();
  T.block(0,3,3,1) = trans;

  T(3,3) = 1.0;

  this->pose = T;
}


SE3::Pose::Pose(Eigen::Matrix4d target) {
  pose = target;

  Eigen::Matrix3d sub_R = target.block(0, 0, 3, 3);
  rotation = SO3::Rotation(sub_R);

  translation = target.block(0,3,3,1);
}


SO3::Rotation SE3::Pose::GetRotation() {
  return rotation;
}


Eigen::Vector3d SE3::Pose::GetTranslation() {
  return translation;
}


Eigen::Matrix4d SE3::Pose::inverse() {
  Eigen::Matrix4d T_inv;
  T_inv.setZero();

  T_inv.block(0,0,3,3) = this->rotation.GetMatrix().transpose();
  T_inv.block(0,3,3,1) = this->rotation.GetMatrix().transpose() * -1. * this->translation;
  T_inv(3,3) = 1.0;  

  return T_inv;
}


Eigen::MatrixXd SE3::Pose::adjoint() {
  Eigen::MatrixXd Ad_T(6,6);
  Ad_T.setZero();

  Eigen::Matrix3d r_hat = SO3::skewSymmetric(this->translation);

  // C - 3x3 matrix
  Ad_T.block(0,0,3,3) = this->rotation.GetMatrix();

  // r^C - 3x3 matrix
  Ad_T.block(0,3,3,3) = r_hat * this->rotation.GetMatrix();

  // C - 3x3 matrix
  Ad_T.block(3,3,3,3) = this->rotation.GetMatrix();

  return Ad_T;
}


Eigen::MatrixXd SE3::Pose::adjoint_inv() {
  Eigen::MatrixXd AdT_inv;
  AdT_inv.setZero();

  // C.T - 3x3 matrix
  // start row: 0, start col: 0
  AdT_inv.block(0,0,3,3) = this->rotation.GetMatrix().transpose();

  // -C.T @ r^ - 3x3 matrix
  // start row: 0, start col: 3
  AdT_inv.block(0,3,3,3) = -1*this->rotation.GetMatrix().transpose()*
                              SO3::skewSymmetric(this->translation);

  // C.T - 3x3 matrix
  // start row: 3, start col: 3
  AdT_inv.block(3,3,3,3) = this->rotation.GetMatrix().transpose();

  return AdT_inv;
}
