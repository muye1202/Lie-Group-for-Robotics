#ifndef SE2_H
#define SE2_H

#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include "lie_lib/so2.h"


namespace SE2 {

  Eigen::Matrix3d skewSymmetric(double angular_vel, Eigen::Vector2d linear_vel);

  Eigen::Vector3d unhat(Eigen::Matrix3d tau_hat);

  Eigen::Matrix3d ExpMap(Eigen::Vector3d tau);

  Eigen::Vector3d LogMap(Eigen::Matrix3d T);

  class Pose{
    public:

      Pose();

      Pose(Eigen::Matrix3d T);

      Pose(Eigen::Vector3d twist);

      Pose(Eigen::Matrix2d R, Eigen::Vector2d t);

      Pose(double theta, Eigen::Vector2d t);

      SE2::Pose operator*(Pose& other);

      Eigen::Matrix3d inverse();

      /**
       * @brief From Micro Lie-theory (159)
       * 
       * @return Eigen::Matrix3d 
       */
      Eigen::Matrix3d adjoint();

      Eigen::Matrix3d GetPose() {return _pose;}

      Eigen::Matrix2d GetRotation() {return _R;}

      Eigen::Vector2d GetTranslation() {return _t;}

      double GetAngle() {return std::acos(_R(0,0));}

      std::pair<double, double> GetTransPair() { return std::make_pair(_t.x(), _t.y());}


    private:

      Eigen::Matrix3d _pose;
      Eigen::Matrix2d _R;
      Eigen::Vector2d _t;
      Eigen::Vector2d _linear;
      double _angular;

  };

}


#endif
