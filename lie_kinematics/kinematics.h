#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Eigen/Dense>
#include "lie_group/se3.h"
#include "lie_group/so3.h"


namespace LieKin
{

  /**
   * @brief Time derivative of Rotation matrix, as defined in
   *        (7.198) in Barfoot's book. Rotation matrix has to
   *        satisfy the constraint C.T * C = I.
   * 
   * @param ang_vel Angular velocity in R3.
   * @param R Rotation matrix at current point.
   * @return Time derivative of rotation matrix.
   */
  Eigen::Matrix3d dRdt(Eigen::Vector3d ang_vel, SO3::Rotation R);


  /**
   * @brief Do projection correction method as outlined in (7.208) to
   *        make sure the integration of rotation matrix is still in
   *        the SO(3) group. The error after correction is 
   *        proportional to the error threshold.
   * 
   * @param C Current rotation matrix from integration.
   * @return Corrected rotation matrix.
   */
  Eigen::Matrix3d RotationCorrection(Eigen::Matrix3d C);


  /**
   * @brief Calculate 3D angular velocity vector from (7.198) in 
   *        Barfoot's book: C_dot*C.T; C.T * C = I constraint applies.
   *        If the rotation is expressed w.r.t the fixed frame, this
   *        velocity will also be expressed in the fixed frame
   * 
   * @param R_dot dR/dt matrix
   * @param R Rotation matrix
   * @return Angular velocity in R3
   */
  Eigen::Vector3d angular_vel(SO3::Rotation R_dot, SO3::Rotation R);


  /**
   * @brief Time derivative of axis-angle vector according to (7.203).
   *        The Jacobian at |phi| = 2*pi*m is not invertible.
   * 
   * @param phi Axis-angle representation of rotation.
   * @param ang_vel Angular velocity.
   * @return dphi/dt vector.
   */
  Eigen::Vector3d dphi_dt(Eigen::Vector3d phi, Eigen::Vector3d ang_vel);


  /**
   * @brief Angular velocity omega calculated acc to (7.202).
   *        If rotation is in world frame, this velocity will
   *        also be in the world frame
   * 
   * @param phi_dot Time derivative of axis-angle vector.
   * @param phi Axis-angle representation of rotation.
   * @return omega = J * dphi/dt
   */
  Eigen::Vector3d angular_vel(Eigen::Vector3d phi_dot, Eigen::Vector3d phi);


  /**
   * @brief Twist vector as defined in (3.71) in Lynch's modern 
   *        robotics book (also in (7.219) in Barfoot's book).
   *        NOTE: if T is {body} expressed in {s}, the Twist calculated
   *        will be in {body}.
   *        
   * 
   * @param T_dot Time derivative of pose T.
   * @param T_inv Inverse of pose T.
   * @return Twist vector as defined in pp.99 in Lynch's book.
   */
  Eigen::VectorXd compute_twist(Eigen::Matrix4d T_dot, Eigen::Matrix4d T_inv);


  /**
   * @brief Time derivative of pose T according to (7.216) in Barfoot's book.
   * 
   * @param twist Twist vector
   * @param T Pose matrix
   * @return T dot matrix 
   */
  Eigen::Matrix4d dT_dt(Eigen::VectorXd twist, Eigen::Matrix4d T);


  /**
   * @brief Convert twist in frame {b} to {s} according to (3.83) in Lynch's book.
   * 
   * @param Ad_Tsb Adjoint matrix of pose Tsb.
   * @param twist_b Twist vector in {b}.
   * @return Twist vector in {s}.
   */
  Eigen::VectorXd convert_twist_frame(Eigen::MatrixXd Ad_Tsb, Eigen::VectorXd twist_b);


  /**
   * @brief Time derivative of p vector in the pose matrix according to
   *        (7.221) in Barfoot's book.
   *        where T = | R p |
   *                  | 0 1 |
   * 
   * @param twist Twist vector
   * @param p translation part of pose T
   * @return p dot vector
   */
  Eigen::Vector3d p_dot(Eigen::VectorXd twist, Eigen::Vector3d p);

}


#endif