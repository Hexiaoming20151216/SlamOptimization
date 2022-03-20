//
// Created by hexiaoming on 2022/3/19.
//

#ifndef SLAM_OPTIMIZATION_OPTIMIZATION_SOLVER_H_
#define SLAM_OPTIMIZATION_OPTIMIZATION_SOLVER_H_

/*
 * 非线性优化求解器
 * 实现GN/LM迭代求解策略
 * */

#include <vector>
#include <eigen3/Eigen/Geometry>
#include "factor.h"
#include "parameter.h"

class Solver {
 public:
  Solver();
  ~Solver();

  ///添加待优化的参数
  void AddParams(const std::shared_ptr<PoseParameter>& parameters);
  ///添加残差因子
  void AddFactor(const Factor::Ptr& factor);
  ///开始求解
  void run(int max_iter_num);

 private:
  double EvaluateJFx(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& J,
                   Eigen::Matrix<double, Eigen::Dynamic, 1>& fx);

 private:
  ///待优化的参数, 一维数组指针
  std::shared_ptr<PoseParameter> parameter_;
//  double * paramters_{};
  ///误差列向量
  Eigen::Matrix<double, Eigen::Dynamic, 1> fx_;
  ///总的雅克比矩阵
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> J_;
  ///H矩阵， H = JT * J
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> H_;

  ///残差因子
  std::vector<Factor::Ptr> factors_;

  ///参数个数
  int param_N_{0};
  ///残差个数
  int res_N_{0};



};

#endif //SLAM_OPTIMIZATION_OPTIMIZATION_SOLVER_H_
