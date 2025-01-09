#include <Eigen/Dense>
#include "se3.h"
#include "so3.h"
#include "uncertainty.h"


Eigen::Matrix3d Lie_Estimation::bracket_single(Eigen::Matrix3d A) {

  return -A.trace() * Eigen::Matrix3d::Identity(3,3) + A;
}


Eigen::Matrix3d Lie_Estimation::bracket_double(Eigen::Matrix3d A, Eigen::Matrix3d B) {

  return bracket_single(A) * bracket_single(B) + bracket_single(A*B);
}


Eigen::MatrixXd Lie_Estimation::merge_poses_cov(SE3::Pose Ta, Eigen::MatrixXd Cov_a, Eigen::MatrixXd Cov_b) {

  Eigen::Matrix3d cova_p, cova_phi, cova_pphi, B_pp, B_pphi, B_phi;
  Eigen::MatrixXd covb_prime, covb_prime_p, covb_prime_pphi,
                  covb_prime_phi;

  // (7.302a)
  cova_p = Cov_a.block(0,0,3,3);
  cova_phi = Cov_a.block(3,3,3,3);
  cova_pphi = Cov_a.block(0,3,3,3);

  // (7.302b)
  covb_prime = Ta.adjoint()*Cov_b*Ta.adjoint().transpose();
  covb_prime_p = covb_prime.block(0,0,3,3);
  covb_prime_pphi = covb_prime.block(0,3,3,3);
  covb_prime_phi = covb_prime.block(3,3,3,3);

  Eigen::MatrixXd A1(6,6), A2_prime(6,6), Beta(6,6);
  A1.setZero(); A2_prime.setZero(); Beta.setZero();

  A1.block(0,0,3,3) = bracket_single(cova_phi);
  A1.block(0,3,3,3) = bracket_single(cova_pphi + cova_pphi.transpose());
  A1.block(3,3,3,3) = bracket_single(cova_phi);

  A2_prime.block(0,0,3,3) = bracket_single(covb_prime_phi);
  A2_prime.block(0,3,3,3) = bracket_single(covb_prime_pphi + covb_prime_pphi.transpose());
  A2_prime.block(3,3,3,3) = bracket_single(covb_prime_phi);

  // (7.303) a to c
  B_pp = bracket_double(cova_phi, covb_prime_p)
          + bracket_double(cova_pphi.transpose(), covb_prime_pphi)
          + bracket_double(cova_pphi, covb_prime_pphi.transpose())
          + bracket_double(cova_p, covb_prime_phi);

  B_pphi = bracket_double(cova_phi, covb_prime_pphi.transpose())
           + bracket_double(cova_pphi.transpose(), covb_prime_phi);

  B_phi = bracket_double(cova_phi, covb_prime_phi);

  // (7.302 e)
  Beta.block(0,0,3,3) = B_pp;
  Beta.block(0,3,3,3) = B_pphi;
  Beta.block(3,0,3,3) = B_pphi.transpose();
  Beta.block(3,3,3,3) = B_phi;

  return Cov_a + covb_prime + 0.25*Beta 
         + (1./12) * (A1*covb_prime + covb_prime*A1.transpose()
                      + A2_prime*Cov_a + Cov_a*A2_prime.transpose());   // (7.304)
}


Eigen::Vector3d rotate_vector(SO3::Rotation R_mean,
                              Eigen::Matrix3d R_cov,
                              Eigen::Vector3d x) {

  Eigen::Matrix3d I = Eigen::Matrix3d::Identity(3,3);
  double tr_cov = R_cov.trace();
  Eigen::Matrix3d coeff_matrix = I + 0.5*(-tr_cov*I + R_cov) 
                                 + (1./24)*( I * (std::pow(tr_cov,2) + (R_cov*R_cov).trace())
                                             - R_cov * (tr_cov*I + 2*R_cov));

  return coeff_matrix * R_mean.GetMatrix() * x;
}


SO3::Rotation Lie_Estimation::left_random_R(SO3::Rotation R_mean, Eigen::Vector3d eps) {

  Eigen::Matrix3d R_left = SO3::ExpMap(eps) * R_mean.GetMatrix();

  return SO3::Rotation(R_left);
}


SO3::Rotation Lie_Estimation::update_R_mean(SO3::Rotation R, SO3::Rotation R_mean) {

  return R*R_mean;
}


Eigen::Matrix3d Lie_Estimation::update_R_covariance(SO3::Rotation R, Eigen::Matrix3d cov_matrix) {

  return R.GetMatrix() * cov_matrix * R.GetMatrix().transpose();
}


SE3::Pose Lie_Estimation::update_T_mean(SE3::Pose update, SE3::Pose old_mean) {

  return update * old_mean;
}


Eigen::MatrixXd Lie_Estimation::update_T_covariance(SE3::Pose update, Eigen::MatrixXd old_cov) {

  Eigen::MatrixXd Ad_T = update.adjoint();

  return Ad_T * old_cov * Ad_T.transpose();
}
