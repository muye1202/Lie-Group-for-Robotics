#ifndef SE3_H
#define SE3_H

#include "so3.h"
#include <Eigen/Dense>


namespace SE3
{

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

      SO3::Rotation GetRotation();

      Eigen::Vector3d GetTranslation();

      Eigen::Matrix4d GetPose() {return pose;}

      Eigen::Matrix4d inverse();

      Eigen::MatrixXd adjoint();

      Eigen::MatrixXd adjoint_inv();

    private:

      Eigen::Matrix4d pose;
      SO3::Rotation rotation;
      Eigen::Vector3d translation;
  };

}


#endif