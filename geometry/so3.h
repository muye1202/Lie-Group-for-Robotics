#ifndef SO3_H
#define SO3_H

#include <iostream>
#include <Eigen/Dense>

namespace SO3
{
  /**
   * @brief Calculate angle of rotation from SO(3) rot matrix.
   *        the angle of rotation will be in the range of |rot| < PI
   * 
   * @param mat - SO(3) rotation matrix 
   * @return Angle of rotation
   */
  double SO3toAngle(Eigen::Matrix3d mat);


  /**
   * @brief Convert so3 skew-symmetric matrix back to R3 vector.
   * 
   * @param matrix
   * @return Vector in R3. 
   */
  Eigen::Vector3d unhat(const Eigen::Matrix3d& matrix);


  /**
   * @brief Return so(3) representation of axis-angle rotation vector.
   * 
   * @param vec 
   * @return 3x3 matrix - the Lie algebra of SO(3).
   */
  Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& vec);


  /**
   * @brief Exponential map of so(3). Convert axis-angle rotation
   *        vector to SO(3) rotation matrix.
   * 
   * @param w
   * @return Rotation matrix in SO(3)
   */
  Eigen::Matrix3d ExpMap(Eigen::Vector3d w);


  /**
   * @brief Compute Left Jacobian of SO(3)
   *        This will be used in computing SE(3) Pose.
   * 
   * @param w 
   * @return Left Jacobian 3x3.
   */
  Eigen::Matrix3d leftJacobian(Eigen::Vector3d w);


  /**
   * @brief Inverse of left Jacobian.
   * 
   * @param w 
   * @return Inverse of Left Jacobian
   */
  Eigen::Matrix3d leftJacobian_inv(Eigen::Vector3d w);


  /**
   * @brief Compute right Jacobian of SO(3)
   *        Reference: State estimation for robotics formula (7.77)
   * 
   * @param w 
   * @return Right Jacobian 3x3
   */
  Eigen::Matrix3d rightJacobian(Eigen::Vector3d w);


  /**
   * @brief Inverse of right Jacobian.
   *        Reference: State estimation for robotics formula (7.76)
   * 
   * @param w 
   * @return Inverse - 3x3
   */
  Eigen::Matrix3d rightJacobian_inv(Eigen::Vector3d w);


  /**
   * @brief J @ J.T where J is the left Jacobian.
   * 
   * @param w 
   * @return J @ J.T
   */
  Eigen::Matrix3d J_JT(Eigen::Vector3d w);


  /**
   * @brief Inverse of J@J.T
   * 
   * @param w 
   * @return (J @ J.T).inv 
   */
  Eigen::Matrix3d J_JT_inv(Eigen::Vector3d w);


  // SO(3) - special orthogonal group representing rotation
  class Rotation{
    public:

      Rotation() {rot.setZero(); mat_rot.setZero();};

      /**
       * @brief Construct a new Rotation object using R3 rotation vector
       *        The Rotation rotation matrix is calculated using exponential map
       * 
       * @param w - rotation vector in R3.
       */
      Rotation(Eigen::Vector3d w);

      Rotation(Eigen::Matrix3d rotation_mat);

      Rotation operator*(Rotation& other) {
        Eigen::Matrix3d result = mat_rot * other.GetMatrix();
        Rotation new_so3(result);

        return new_so3;
      }

      Rotation transpose();

      Rotation inverse();

      Eigen::Matrix3d GetMatrix();

      double get(int row, int col);

    private:

      Eigen::Vector3d rot;
      Eigen::Matrix3d mat_rot;
  };


  /**
   * @brief Log map from SO(3) to so(3), return axis-angle
   *        representation of rotation.
   *        Reference: Micro Lie Theory for Robotics - Appendix B
   *                   Exploring SO(3) logarithmic map: degeneracies and derivatives By Z. Nurlanov
   * 
   * @param R 
   * @return Axis-angle rotation vector.
   */
  Eigen::Vector3d LogMap(Eigen::Matrix3d R);

}


#endif
