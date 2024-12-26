#ifndef SE3_H
#define SE3_H

#include "so3.h"
#include <Eigen/Dense>


namespace SE3
{

  /**
   * @brief Lie Algebra for 4d vector. As defined in page 221 in 
   *        Barfoot's book, used in pose derivative in (7.187).
   * 
   * @param p 4d vector
   * @return 4x6 matrix for Lie Algebra.
   */
  Eigen::MatrixXd LieAlgebra_4d(const Eigen::Vector4d& p);

  /**
   * @brief Hat operator for 6d pose vector - [pos rot]
   * 
   * @param eksi 6d pose vector
   * @return matrix in se(3)
   */
  Eigen::Matrix4d skewSymmetric(const Eigen::VectorXd& eksi);


  /**
   * @brief Exponential Map for se(3) goes from se(3) to SE(3)
   * 
   * @param eksi 6d vector [rou phi] rou is translation phi is rotation
   *             each vector is 3d.
   * @return Pose matrix in SE(3)
   */
  Eigen::Matrix4d ExpMap(Eigen::VectorXd eksi);


  /**
   * @brief left and right Q block matrix as defined in
   *        equation 7.86 in State Estimation for Robotics book.
   * 
   * @param eksi 6d vector containing rot and trans vector.
   * 
   * @return Ql and Qr - 6x6 matrices. 
   */
  Eigen::Matrix3d leftQ(Eigen::VectorXd eksi);

  Eigen::Matrix3d rightQ(Eigen::VectorXd eksi);


  /**
   * @brief Left Jacobian of SE(3) pose matrix as defined in
   *        eqn. (7.85) in State Estimation for Robotics book.
   * 
   * @param eksi 6d vector containing rot and trans.
   * 
   * @return J_left 6x6 matrix 
   */
  Eigen::MatrixXd leftJacobian(Eigen::VectorXd eksi);


  /**
   * @brief Right Jacobian and Jr = Jl(-eksi)
   * 
   * @param eksi 6d vector containing rot and trans.
   * 
   * @return J_r 6x6 matrix
   */
  Eigen::MatrixXd rightJacobian(Eigen::VectorXd eksi);


  /**
   * @brief Inverse of left Jacobian of SE(3) elements.
   *        As given in (7.95) in Barfoot's text.
   * 
   * @param eksi 6d vector containing translation and rotation.
   * @return 6x6 inverse of J_left.
   */
  Eigen::MatrixXd leftJacobian_inv(Eigen::VectorXd eksi);


  /**
   * @brief Inverse of right Jacobian.
   * 
   * @param eksi 6d vector containing translation and rotation.
   * @return 6x6 inverse of J_left.
   */
  Eigen::MatrixXd rightJacobian_inv(Eigen::VectorXd eksi);


  class Pose {
    public:

      Pose(Eigen::Vector3d rot, Eigen::Vector3d trans);

      Pose(Eigen::Matrix3d R, Eigen::Vector3d trans);

      Pose(SO3::Rotation R, Eigen::Vector3d trans);

      Pose(Eigen::Matrix4d target);

      Pose operator*(Pose& other) {
        Eigen::Matrix4d result = pose * other.GetPose();
        Pose new_so3(result);

        return new_so3;
      }

      SO3::Rotation GetRotation() {return rotation;};

      Eigen::Vector3d GetTranslation() {return translation;};

      Eigen::Matrix4d GetPose() {return pose;}

      Eigen::Vector3d GetAxisAngle() {return axis_angle;}

      Eigen::VectorXd GetPoseVector();

      Eigen::Matrix4d inverse();

      Eigen::MatrixXd adjoint();

      Eigen::MatrixXd adjoint_inv();

    private:

      Eigen::Matrix4d pose;
      SO3::Rotation rotation;
      Eigen::Vector3d axis_angle;
      Eigen::Vector3d translation;
  };


  /**
   * @brief Derivative of T*p w.r.t Eksi vector representing translation and
   *        rotation. As defined in (7.187) in Barfoot's book. 
   * 
   * @param T SE(3) matrix representing pose.
   * @param p Homogeneous coordinate in R4.
   * @return 4x4 matrix.
   */
  Eigen::MatrixXd PoseJacobian(SE3::Pose T, Eigen::Vector4d p);


  /**
   * @brief Derivative w.r.t perturbation of SE(3) pose without the need
   *        of left Jacobian. Defined in (7.189) in Barfoot's book.
   * 
   * @param T SE(3) matrix representing pose.
   * @param p Homogeneous coordinate in R4.
   * @return 4x4 matrix.
   */
  Eigen::Matrix4d leftPerturbDerivative(SE3::Pose T, Eigen::Vector4d p);

}


#endif