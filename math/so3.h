//
// Created by hexiaoming on 2022/3/19.
//

#ifndef SLAM_OPTIMIZATION_SO3_H
#define SLAM_OPTIMIZATION_SO3_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include "common.h"

/*
 *单位四元数表旋转，实现流形上的加法，用于优化中的更新
 * */

namespace slamopt{

template <typename T>
class SO3{
 public:
  using Quat = Eigen::Quaternion<T>;
  using Vec3 = Eigen::Matrix<T,3,1>;

  SO3(Eigen::Quaternion<T> quaternion):quaternion_(std::move(quaternion)){}

  Eigen::Quaternion<double> Plus(const Vec3& delta){
    return exp(delta) * quaternion_;
  }

 private:
  Eigen::Quaternion<double> quaternion_;
};

using SO3d = SO3<double>;
}
#endif //SLAM_OPTIMIZATION_SO3_H
