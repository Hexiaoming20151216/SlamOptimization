//
// Created by hexiaoming on 2022/3/19.
//

#ifndef SLAM_OPTIMIZATION_OPTIMIZATION_FACTOR_H_
#define SLAM_OPTIMIZATION_OPTIMIZATION_FACTOR_H_

#include <eigen3/Eigen/Geometry>
#include <memory>
#include "../math/common.h"

/*
 * 残差因子类
 * 残差为数组指针，长度为残差个数固定
 * 残差对参数的雅克比为数组指针，长度为残差长度*参数长度
 * */

class Factor {
 public:
  using Ptr = std::shared_ptr<Factor>;
  Factor(int Res_N/*残差个数*/, int Param_N/*参数个数*/):
      residuals_(new double[Res_N]),
      Res_N_(Res_N),
      J_(new double[Res_N*Param_N]){

  }
  virtual ~Factor(){
    delete[] residuals_;
    delete[] J_;
  }

  void SetParams(double* param){
    parameters_= param;
  }

  int GetResN(){
    return Res_N_;
  }

  double * GetResiduals(){
    return residuals_;
  }

  double * GetJacobians(){
    return J_;
  }

  ///求解残差、雅克比的方法,返回当前残差和
  virtual double Evaluate() = 0;
 protected:
  ///参数指针
  double* parameters_{};

  ///残差数组，根据参数变化而变化，优化过程中应越来越小
  double* residuals_;
  const int Res_N_;

  ///残差对参数的雅克比
  double* J_;
};


/*
 * 3d point to point icp
 * 残差为点之间的欧氏距离
 * */
class PointPointFactor: public Factor{
 public:
  PointPointFactor(const Eigen::Vector3d& point_0,             ///单个点，在激光坐标系下的坐标点
                   const Eigen::Vector3d& point_1)
                   : point_0_(point_0), point_1_(point_1), Factor(3,6){
//    residuals_.reserve(1);
  }
  ~PointPointFactor() override{};

  double Evaluate() override {
    Eigen::Map<Eigen::Quaterniond> rotation(parameters_);
    Eigen::Map<Eigen::Vector3d> trans(parameters_+4);

    Eigen::Map<Eigen::Vector3d> res(residuals_);

    ///将激光系下的点转到世界系下
    Eigen::Vector3d point0_w = rotation*point_0_ + trans;
    ///残差求解
    res = point0_w - point_1_;

    ///雅克比求解
    ///残差一维，全局参数7维，局部参数6维，这里对局部参数求导
    Eigen::Map<Eigen::Matrix<double, 3, 6>> Jacobian(J_);
    Jacobian.block(0,0,3,3) = -slamopt::hat(point0_w);
    Jacobian.block(0,3,3,3) = Eigen::Matrix3d::Identity();

    return res.norm();
  }

 private:
  Eigen::Vector3d point_0_;             ///单个点，在激光坐标系下的坐标点
  Eigen::Vector3d point_1_;             ///单个点，在世界系下的坐标点
};

#endif //SLAM_OPTIMIZATION_OPTIMIZATION_FACTOR_H_
