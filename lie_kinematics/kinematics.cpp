#include "kinematics.h"
#include "lie_group/so3.h"
#include "lie_group/se3.h"


Eigen::Matrix3d LieKin::dRdt(Eigen::Vector3d ang_vel, SO3::Rotation R) {

  Eigen::Matrix3d omega_hat = SO3::skewSymmetric(ang_vel);

  return omega_hat * R.GetMatrix();
}


Eigen::Matrix3d LieKin::RotationCorrection(Eigen::Matrix3d C) {

  // First solve for eigen values and vectors
  Eigen::Matrix3d CCT = C * C.transpose();
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(CCT);

  if (solver.info() != Eigen::Success) {
    std::cerr << "Eigenvalue decomposition failed!" << std::endl;
    exit(1);
  }

  Eigen::VectorXd eigenvalues = solver.eigenvalues();
  Eigen::MatrixXd eigenvectors = solver.eigenvectors();

  Eigen::MatrixXd sqrtEigenvalues = eigenvalues.array().sqrt().matrix().asDiagonal();
  Eigen::MatrixXd sqrtA = eigenvectors * sqrtEigenvalues * eigenvectors.transpose();

  Eigen::MatrixXd sqrt_inv = sqrtA.inverse();

  return sqrt_inv * C;
}


Eigen::Vector3d LieKin::angular_vel(SO3::Rotation R_dot, SO3::Rotation R) {

  // Check if orthogonality is satisfied
  Eigen::Matrix3d C = R.GetMatrix();
  Eigen::Matrix3d CT_C = C.transpose() * R.GetMatrix();
  if (!Eigen::Matrix3d::Identity(3,3).isApprox(CT_C, 1e-4)) {
    C = LieKin::RotationCorrection(C);
  }

  Eigen::Matrix3d Cdot_CT = R_dot.GetMatrix() * C.transpose();

  return SO3::unhat(Cdot_CT);
}


Eigen::Vector3d LieKin::angular_vel(Eigen::Vector3d phi_dot, Eigen::Vector3d phi) {

  // Compute the left Jacobian
  Eigen::Matrix3d J_left = SO3::leftJacobian(phi);

  return J_left * phi_dot;
}


Eigen::Vector3d LieKin::dphi_dt(Eigen::Vector3d phi, Eigen::Vector3d ang_vel) {

  Eigen::Matrix3d J_left_inv = SO3::leftJacobian_inv(phi);

  return J_left_inv * ang_vel;
}


Eigen::VectorXd LieKin::compute_twist(Eigen::Matrix4d T_dot, Eigen::Matrix4d T_inv) {

  Eigen::Matrix4d twist_hat = T_dot * T_inv;

  return SE3::unhat_6d(twist_hat);
}


Eigen::Matrix4d LieKin::dT_dt(Eigen::VectorXd twist, Eigen::Matrix4d T) {

  Eigen::Matrix4d Tdot = SE3::skewSymmetric(twist) * T;

  return Tdot;
}


Eigen::VectorXd LieKin::convert_twist_frame(Eigen::MatrixXd Ad_Tsb, Eigen::VectorXd twist_b) {

  return Ad_Tsb * twist_b;
}


Eigen::Vector3d LieKin::p_dot(Eigen::VectorXd twist, Eigen::Vector3d p) {

  Eigen::Vector3d v = twist.segment<3>(0);
  Eigen::Vector3d omega = twist.segment<3>(3);
  Eigen::Matrix3d p_hat = -1*SO3::skewSymmetric(p);

  return v + p_hat*omega;
}
