#ifndef SO2_H
#define SO2_H

#include <iostream>
#include <Eigen/Dense>


namespace SO2
{

  double unhat(Eigen::Matrix2d theta_hat);


  Eigen::Matrix2d skewSymmetric(double theta);


  Eigen::Matrix2d ExpMap(double theta);


  double LogMap(Eigen::Matrix2d R);


  double ProductLogMap(Eigen::Matrix2d R1, Eigen::Matrix2d R2);


  class Rotation{

    public:

      /**
       * @brief Construct a default identity rotation
       * 
       */
      Rotation() {
        _theta = 0.0;
        _R = Eigen::Matrix2d::Identity(2,2);
      };

      Rotation(double theta) {
        _theta = theta;
        _R = SO2::ExpMap(theta);
      };

      Rotation(Eigen::Matrix2d R) {
        _theta = SO2::LogMap(R);
        _R = R;
      };

      Rotation inverse() {

        return Rotation(_R.transpose());
      };

      Eigen::Vector2d JRv_R(Rotation R, Eigen::Vector2d v) {

        return R._R * skewSymmetric(1.0) * v;
      }

      Eigen::Matrix2d JRv_v() {

        return _R;
      }

      double adjoint() {
        return 1.0;
      }

      double JRinv_R() {
        return -1.0;
      }

      double JQR_Q() {
        return 1.0;
      }

      double JQR_R() {
        return 1.0;
      }

      double right_Jacobian() {
        return 1.0;
      }

      double left_Jacobian() {
        return 1.0;
      }

      double JRtheta_R() {
        return 1.0;
      }

      double JRtheta_theta() {
        return 1.0;
      }

      double JQminusR_Q() {
        return 1.0;
      }

      double JQminusR_R() {
        return -1.0;
      }


      Rotation operator*(Rotation& other) {

        return Rotation(_theta + other._theta);
      }

      double operator-(Rotation& other) {

        return _theta - other._theta;
      }

    private:

      Eigen::Matrix2d _R;
      double _theta;

  };

}


#endif