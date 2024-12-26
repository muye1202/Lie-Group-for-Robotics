#include "se3.h"
#include <Eigen/Dense>


Eigen::MatrixXd SE3::LieAlgebra_4d(const Eigen::Vector4d& p) {

  Eigen::MatrixXd result(4,6);
  result.setZero();

  Eigen::Vector3d eps;
  eps << p.x(), p.y(), p.z();

  result.block(0,0,3,3) = p.w() * Eigen::Matrix3d::Identity(3,3);
  result.block(0,3,3,3) = -1*SO3::skewSymmetric(eps);

  return result;
}


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

///////////// Constructors /////////////

SE3::Pose::Pose(Eigen::Vector3d rot, Eigen::Vector3d trans) {
  rotation = SO3::Rotation(rot);
  translation = trans;
  axis_angle = rot;

  Eigen::Matrix4d T;
  T.block(0,0,3,3) = rotation.GetMatrix();
  T.block(0,3,3,1) = trans;

  T(3,3) = 1.0;

  this->pose = T;
}


SE3::Pose::Pose(Eigen::Matrix3d R, Eigen::Vector3d trans) {
  rotation = SO3::Rotation(R);
  translation = trans;

  // Get axis-angle rot from R
  Eigen::Vector3d rot_vec = SO3::LogMap(R);
  axis_angle = rot_vec;

  Eigen::Matrix4d T;
  T.block(0,0,3,3) = R;
  T.block(0,3,3,1) = trans;

  T(3,3) = 1.0;

  this->pose = T;
}


SE3::Pose::Pose(SO3::Rotation R, Eigen::Vector3d trans) {
  rotation = R;
  translation = trans;

  // Get axis-angle rot from R
  Eigen::Vector3d rot_vec = SO3::LogMap(R.GetMatrix());
  axis_angle = rot_vec;

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

  // Get axis-angle rot from R
  Eigen::Vector3d rot_vec = SO3::LogMap(sub_R);
  axis_angle = rot_vec;

  translation = target.block(0,3,3,1);
}

/////////////////////////////////////////

Eigen::VectorXd SE3::Pose::GetPoseVector() {

  // 6d pose vector eksi
  Eigen::VectorXd eksi(6);
  eksi << this->translation, this->axis_angle;

  return eksi;
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

  Eigen::MatrixXd AdT_inv(6,6);
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


Eigen::MatrixXd SE3::leftJacobian(Eigen::VectorXd eksi) {

  Eigen::Vector3d rot = eksi.segment<3>(0);
  Eigen::Vector3d p = eksi.segment<3>(3);

  // Get Jl block where Jl is Jacobian of SO(3)
  Eigen::Matrix3d Jl = SO3::leftJacobian(rot);

  Eigen::Matrix3d Q_left = SE3::leftQ(eksi);

  Eigen::MatrixXd left_J(6,6);
  left_J.setZero();

  left_J.block(0,0,3,3) = Jl;
  left_J.block(0,3,3,3) = Q_left;
  left_J.block(3,3,3,3) = Jl;

  return left_J;
}


Eigen::MatrixXd SE3::rightJacobian(Eigen::VectorXd eksi) {

  Eigen::Vector3d rot = eksi.segment<3>(0);
  Eigen::Vector3d p = eksi.segment<3>(3);

  // Get Jl block where Jl is Jacobian of SO(3)
  Eigen::Matrix3d Jr_so3 = SO3::rightJacobian(rot);

  Eigen::Matrix3d Q_r = SE3::rightQ(eksi);

  Eigen::MatrixXd Jr(6,6);
  Jr.setZero();

  Jr.block(0,0,3,3) = Jr_so3;
  Jr.block(0,3,3,3) = Q_r;
  Jr.block(3,3,3,3) = Jr_so3;

  return Jr;
}


Eigen::MatrixXd SE3::leftJacobian_inv(Eigen::VectorXd eksi) {

  Eigen::Vector3d rot = eksi.segment<3>(0);
  Eigen::Vector3d p = eksi.segment<3>(3);

  Eigen::MatrixXd Jl_inv(6,6);
  Jl_inv.setZero();

  Eigen::Matrix3d SO3_Jl_inv = SO3::leftJacobian_inv(rot);
  Eigen::Matrix3d Ql = SE3::leftQ(eksi);

  Jl_inv.block(0,0,3,3) = SO3_Jl_inv;
  Jl_inv.block(0,3,3,3) = -1*SO3_Jl_inv*Ql*SO3_Jl_inv;
  Jl_inv.block(3,3,3,3) = SO3_Jl_inv;

  return Jl_inv;
}


Eigen::MatrixXd SE3::rightJacobian_inv(Eigen::VectorXd eksi) {

  Eigen::Vector3d rot = eksi.segment<3>(0);
  Eigen::Vector3d p = eksi.segment<3>(3);

  Eigen::MatrixXd Jr_inv(6,6);
  Jr_inv.setZero();

  Eigen::Matrix3d SO3_Jr_inv = SO3::rightJacobian_inv(rot);
  Eigen::Matrix3d Qr = SE3::rightQ(eksi);

  Jr_inv.block(0,0,3,3) = SO3_Jr_inv;
  Jr_inv.block(0,3,3,3) = -1*SO3_Jr_inv*Qr*SO3_Jr_inv;
  Jr_inv.block(3,3,3,3) = SO3_Jr_inv;

  return Jr_inv;
}


// Formula (7.86b) in Barfoot's book
Eigen::Matrix3d SE3::leftQ(Eigen::VectorXd eksi) {

  Eigen::Vector3d rot = eksi.segment<3>(0);
  Eigen::Vector3d p = eksi.segment<3>(3);

  // rou^ matrix where rou is translation
  Eigen::Matrix3d p_hat = SO3::skewSymmetric(p);

  // scalar rotation angle
  double phi = rot.norm();

  // phi^ matrix where phi is axis-angle vector
  Eigen::Matrix3d phi_hat = SO3::skewSymmetric(rot);

  // 2nd term in (b)
  Eigen::Matrix3d second_term = ((phi - std::sin(phi))/std::pow(phi, 3)) * 
                                (phi_hat*p_hat + p_hat*phi_hat + phi_hat*p_hat*phi_hat);

  Eigen::Matrix3d third_term = ((std::pow(phi,2) + 2*std::cos(phi) - 2) / (2*std::pow(phi, 4))) * 
                               (phi_hat*phi_hat*p_hat + p_hat*phi_hat*phi_hat - 3*phi_hat*p_hat*phi_hat);

  Eigen::Matrix3d fourth_term = ((2*phi-3*std::sin(phi)+phi*std::cos(phi)) / 2*std::pow(phi, 5)) * 
                                (phi_hat*p_hat*phi_hat*phi_hat + phi_hat*phi_hat*p_hat*phi_hat);

  return 0.5*phi_hat + second_term + third_term + fourth_term;
}


// Q_r = Q_l(-eksi) as in (7.86c)
Eigen::Matrix3d SE3::rightQ(Eigen::VectorXd eksi) {
  
  return SE3::leftQ(-1*eksi);
}


Eigen::MatrixXd SE3::PoseJacobian(SE3::Pose T, Eigen::Vector4d p) {

  Eigen::Vector3d angle_axis = T.GetAxisAngle();
  Eigen::MatrixXd left_J = SE3::leftJacobian(T.GetPoseVector());

  Eigen::Vector4d T_p = T.GetPose() * p;
  Eigen::MatrixXd lie_algebra_Tp = SE3::LieAlgebra_4d(T_p);

  // DEBUG: need to check the dimension 
  Eigen::MatrixXd result = lie_algebra_Tp * left_J;

  return result;
}


Eigen::Matrix4d SE3::leftPerturbDerivative(SE3::Pose T, Eigen::Vector4d p) {

  Eigen::Vector4d T_p = T.GetPose() * p;
  Eigen::MatrixXd lie_algebra_Tp = SE3::LieAlgebra_4d(T_p);   // 4x6 matrix

  return lie_algebra_Tp;
}
