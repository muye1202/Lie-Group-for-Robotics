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
  SO3::Rotation dRdt(Eigen::Vector3d ang_vel, SO3::Rotation R);


  /**
   * @brief Calculate 3D angular velocity vector from (7.198) in 
   *        Barfoot's book; C.T * C = I constraint applies.
   * 
   * @param R_dot dR/dt matrix
   * @param R Rotation matrix
   * @return Angular velocity in R3
   */
  Eigen::Vector3d Angular_Velocity(SO3::Rotation R_dot, SO3::Rotation R);


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
   * 
   * @param phi_dot Time derivative of axis-angle vector.
   * @param phi Axis-angle representation of rotation.
   * @return omega = J * dphi/dt
   */
  Eigen::Vector3d Angular_Velocity(Eigen::Vector3d phi_dot, Eigen::Vector3d phi);

}


#endif