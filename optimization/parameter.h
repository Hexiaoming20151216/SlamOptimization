//
// Created by hexiaoming on 2022/3/20.
//

#ifndef SLAM_OPTIMIZATION_OPTIMIZATION_PARAMETER_H_
#define SLAM_OPTIMIZATION_OPTIMIZATION_PARAMETER_H_

#include "../math/so3.h"

class Parameter {
 public:
  Parameter(int global_size,int local_size)
      :global_size_(global_size),local_size_(local_size){
    parameters_ = new double[7];
  }

  virtual ~Parameter(){
    delete[] parameters_;
  }

  double* GetParams(){
    return parameters_;
  };

  int GetGlobalSize(){
    return global_size_;
  }
  int GetLocalSize(){
    return local_size_;
  }

  ///参数更新方法
  virtual void Plus(double* delta) = 0;

  ///返回上一次状态，舍去最新一次的参数更新
  virtual void BackToLastState() = 0;

 protected:
  ///参数指针
  double* parameters_{};
  ///manifold上的参数维数
  int global_size_;
  ///tangent上的参数维数
  int local_size_;

};

/*
 * 3d位姿参数，旋转+平移
 * */
class PoseParameter: public Parameter{
 public:
  PoseParameter(const Eigen::Quaterniond& q, const Eigen::Vector3d& t)
      : last_rotation_(q), last_tasnlation_(t), Parameter(7, 6){
    ///参数前四个为旋转四元数，后三个为平移向量
    Eigen::Map<Eigen::Quaterniond> rotation(parameters_);
    Eigen::Map<Eigen::Vector3d> trans(parameters_+4);

    rotation = q;
    trans = t;
  }

  ~PoseParameter(){};

  void Plus(double* delta) override {
    Eigen::Map<Eigen::Quaterniond> rotation(parameters_);
    Eigen::Map<Eigen::Vector3d> trans(parameters_+4);
    Eigen::Map<Eigen::Matrix<double,6,1>> eigen_delta(delta);

    slamopt::SO3d r_so3(rotation);

    rotation = r_so3.Plus(eigen_delta.block(0,0,3,1));
    trans = trans + eigen_delta.block(3,0,3,1);

    last_rotation_ = rotation;
    last_tasnlation_ = trans;
  }

  void BackToLastState(){
    Eigen::Map<Eigen::Quaterniond> rotation(parameters_);
    Eigen::Map<Eigen::Vector3d> trans(parameters_+4);

    rotation = last_rotation_;
    trans = last_tasnlation_;
  }
 private:
  Eigen::Quaterniond last_rotation_;
  Eigen::Vector3d last_tasnlation_;
};

#endif //SLAM_OPTIMIZATION_OPTIMIZATION_PARAMETER_H_
