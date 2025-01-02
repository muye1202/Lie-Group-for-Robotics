#ifndef UNCERTAINTY_PROP_H
#define UNCERTAINTY_PROP_H

#include <Eigen/Dense>
#include "se3.h"
#include "so3.h"


namespace LIE_RANDOM
{

  /**
   * @brief SO(3) as a Gaussian random variable defined by (7.264) in Barfoot's book.
   * 
   * @param R_mean Mean rotation matrix (similar to mean mu in Gaussian distrib.)
   * @param eps Noise vector in R3.
   * @return left representation of Gaussian random variable rotation.
   */
  SO3::Rotation left_random_R(SO3::Rotation R_mean, Eigen::Vector3d eps);


  /**
   * @brief Update rotation mean according to (7.277) in Barfoot's book.
   * 
   * @param R Rotation amount.
   * @param R_mean Old mean rotation matrix.
   * @return Updated rotation matrix.
   */
  SO3::Rotation update_R_mean(SO3::Rotation R, SO3::Rotation R_mean);


  /**
   * @brief Update covariance matrix after rotation R - (7.277)
   * 
   * @param R Rotation amount.
   * @param cov_matrix Old covariance matrix.
   * @return New covariance matrix after rotation by R.
   */
  Eigen::Matrix3d update_R_covariance(SO3::Rotation R, Eigen::Matrix3d cov_matrix);


  /**
   * @brief Update mean of the Gaussian variable pose matrix - (7.283)
   * 
   * @param update Update quantity for pose.
   * @param old_mean Old mean of Gaussian pose distribution.
   * @return New mean of Gaussian pose distribution.
   */
  SE3::Pose update_T_mean(SE3::Pose update, SE3::Pose old_mean);


  /**
   * @brief Update covariance of the Gaussian pose distribution - (7.283)
   * 
   * @param update Update quantity for pose.
   * @param old_cov Original covariance of Gaussian distribution - 6x6
   * @return Updated covariance matrix - 6x6
   */
  Eigen::MatrixXd update_T_covariance(SE3::Pose update, Eigen::MatrixXd old_cov);

}



# endif