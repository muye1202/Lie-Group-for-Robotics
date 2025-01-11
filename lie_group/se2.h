#ifndef SE2_H
#define SE2_H

#include <iostream>
#include <Eigen/Dense>
#include "so2.h"


namespace SE2 {

  Eigen::Matrix3d skewSymmetric(double angular_vel, Eigen::Vector2d linear_vel);

  Eigen::Vector3d unhat(Eigen::Matrix3d tau_hat);

  Eigen::Matrix3d ExpMap(Eigen::Vector3d tau);

  class Pose{
    public:

      Pose() {
        _pose = Eigen::Matrix3d::Identity(3,3);
        _R = Eigen::Matrix2d::Identity(2,2);
        _t = Eigen::Vector2d::Zero();
        _angular_pos = 0.0;
      }

      Pose(Eigen::Matrix2d R, Eigen::Vector2d t);

      Pose(double theta, Eigen::Vector2d t);

      Eigen::Matrix3d inverse();

      Eigen::Matrix3d operator*(Pose& other);


    private:

      Eigen::Matrix3d _pose;
      Eigen::Matrix2d _R;
      Eigen::Vector2d _t;
      double _angular_pos;

  };

  Eigen::Vector3d LogMap(SE2::Pose T);

}


#endif
