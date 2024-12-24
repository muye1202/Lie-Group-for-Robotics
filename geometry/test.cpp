#include "so3.h"
#include "se3.h"
#include <Eigen/Dense>
#include <iostream>


double PI = 3.1415926;


int main() {

  Eigen::Vector3d x_rot;
  x_rot << PI/2., 0.0, 0.0;
  Eigen::Vector3d y_rot;
  y_rot << 0.0, PI/2., 0.0;
  Eigen::Vector3d z_rot;
  z_rot << 0.0, 0.0, PI/2.;

  Eigen::Vector3d p1;
  p1 << 1.0, 2.0, 3.0;

  Eigen::Matrix3d m1 {
    {0.0, -1.0, 1.0},
    {1.0, 0.0, -1.0},
    {-1.0, 1.0, 0.0}
  };

  Eigen::Matrix3d ref_Rx;
  ref_Rx << 1.0, 0.0, 0.0,
            0.0, std::cos(PI/2.), -std::sin(PI/2.),
            0.0, std::sin(PI/2.), std::cos(PI/2.);

  SO3::Rotation Rx(x_rot);
  std::cout << "rotation 45 degree in x-axis:\n";
  std::cout << Rx.GetMatrix();
  std::cout << std::endl;

  std::cout << "transpose of Rx:\n";
  std::cout << Rx.transpose().GetMatrix();
  std::cout << std::endl;

  std::cout << "inverse of Rx:\n";
  std::cout << Rx.inverse().GetMatrix();
  std::cout << std::endl;

  // if (ref_Rx.isApprox(Rx.GetMatrix())) {
  //   std::cout << "implementation of Rx is correct!\n";
  // }

  // Eigen::Vector3d vec = SO3::LogMap(Rx.GetMatrix());
  // std::cout << "\noriginal rot vec:\n";
  // std::cout << vec << std::endl;

  // Eigen::Matrix3d ref_Ry;
  // ref_Ry << std::cos(PI/2.), 0.0, std::sin(PI/2.),
  //           0.0, 1.0, 0.0,
  //           -std::sin(PI/2.), 0.0, std::cos(PI/2.);

  // SO3::Rotation Ry(y_rot);
  // std::cout << "\nrotation 45 degree in y-axis:\n";
  // std::cout << Ry.GetMatrix();
  // std::cout << std::endl;

  // if (ref_Ry.isApprox(Ry.GetMatrix())) {
  //   std::cout << "implementation of Ry is correct!\n";
  // }

  // Eigen::Matrix3d ref_Rz;
  // ref_Rz << std::cos(PI/2.), -std::sin(PI/2.), 0.0,
  //           std::sin(PI/2.), std::cos(PI/2.), 0.0,
  //           0.0, 0.0, 1.0;

  // SO3::Rotation Rz(z_rot);
  // std::cout << "\nrotation 45 degree in z-axis:\n";
  // std::cout << Rz.GetMatrix();
  // std::cout << std::endl;

  // if (ref_Rz.isApprox(Rz.GetMatrix())) {
  //   std::cout << "implementation of Rz is correct!\n";
  // }

  // Eigen::Vector3d xyz_rot;
  // xyz_rot << 0.5, 1.0, 1.0;

  // Eigen::Matrix3d ref_Rxyz;
  // ref_Rxyz << 0.173989, -0.458494,  0.8715,
  //             0.8715,  0.483743,  0.0805074,
  //             -0.458494,  0.745504,  0.483743;

  // SO3::Rotation R(xyz_rot);
  // std::cout << "\nrotation in 3 axis:\n";
  // std::cout << R.GetMatrix();
  // std::cout << std::endl;

  // Eigen::Vector3d vec1 = SO3::LogMap(R.GetMatrix());
  // std::cout << "\noriginal rot vec:\n";
  // std::cout << vec1 << std::endl;

}
