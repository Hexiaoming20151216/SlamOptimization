//
// Created by hexiaoming on 2022/3/19.
//

#ifndef SLAM_OPTIMIZATION_COMMON_H
#define SLAM_OPTIMIZATION_COMMON_H

#include <math.h>
#include <eigen3/Eigen/Core>

namespace slamopt{

///旋转四元数到切平面增量的对数映射
template <typename T>
Eigen::Matrix<T,3,1> log(const Eigen::Quaternion<T>& unit_quaternion) {
  T squared_n = unit_quaternion.vec().squaredNorm();
  T w = unit_quaternion.w();
  T two_atan_nbyw_by_n;

  if (squared_n < 1e-10 * 1e-10) {
    T squared_w = w * w;
    two_atan_nbyw_by_n =
        T(2) / w - T(2) * (squared_n) / (w * squared_w);
  } else {
    T n = sqrt(squared_n);
    if (abs(w) < 1e-10) {
      if (w > T(0)) {
        two_atan_nbyw_by_n = M_PI / n;
      } else {
        two_atan_nbyw_by_n = -M_PI / n;
      }
    } else {
      two_atan_nbyw_by_n = T(2) * std::atan(n / w) / n;
    }
  }

  return two_atan_nbyw_by_n * unit_quaternion.vec();
}

///指数映射， 角轴增量（切平面）到旋转四元数（流形空间）
// q = [cos(theta/2), [u]sin(theta/2)]
template <typename T>
Eigen::Quaternion<T> exp(const Eigen::Matrix<T, 3, 1>& angle_axis){
  T theta = T(0); //角度
  T theta_sq = angle_axis.squaredNorm();

  T imag_factor; //虚部
  T real_factor; //实部
  if (theta_sq < 1e-10 * 1e-10) {
    theta = T(0);
    T theta_po4 = theta_sq * theta_sq;
    imag_factor = T(0.5) - T(1.0 / 48.0) * theta_sq +
        T(1.0 / 3840.0) * theta_po4;
    real_factor = T(1) - T(1.0 / 8.0) * theta_sq +
        T(1.0 / 384.0) * theta_po4;
  } else {
    theta = sqrt(theta_sq);
    T half_theta = T(0.5) * (theta);
    T sin_half_theta = sin(half_theta);
    imag_factor = sin_half_theta / (theta);
    real_factor = cos(half_theta);
  }

  Eigen::Quaternion<T> q;
  q.w() = real_factor;
  q.x() = imag_factor * angle_axis.x();
  q.y() = imag_factor * angle_axis.x();
  q.z() = imag_factor * angle_axis.x();

  return q;
}

template <typename T>
Eigen::Matrix<T,3,3> hat(const Eigen::Matrix<T, 3, 1>& omega){
  Eigen::Matrix<T,3,3> Omega;
  // clang-format off
  Omega <<
        T(0), -omega(2),  omega(1),
      omega(2), T(0), -omega(0),
      -omega(1),  omega(0), T(0);
  // clang-format on
  return Omega;
}

}
#endif //SLAM_OPTIMIZATION_COMMON_H
