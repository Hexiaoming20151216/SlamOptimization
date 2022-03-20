//
// Created by hexiaoming on 2022/3/19.
//

#include <iostream>
#include "solver.h"

Solver::Solver(){

}

Solver::~Solver(){
//  delete[] paramters_;
}

///添加待优化的参数
void Solver::AddParams(const std::shared_ptr<PoseParameter>& parameter){
  parameter_ = parameter;
//  paramters_ = parameters;
  param_N_ = parameter->GetLocalSize();
}

void Solver::AddFactor(const Factor::Ptr& factor){
  factor->SetParams(parameter_->GetParams());
  factors_.emplace_back(factor);
  res_N_ += factor->GetResN();
}

void Solver::run(int max_iter_num){
  bool finished = false;
  int iter_count = 0;
  ///总的雅克比矩阵
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>  J(res_N_, param_N_);
  ///误差列向量
  Eigen::Matrix<double, Eigen::Dynamic, 1>  fx(res_N_, 1);
  ///H矩阵， H = JT * J
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>  H(param_N_, param_N_);
  ///b = -JT * fx
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>  _JtF(param_N_, 1);
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>  last_JtF(param_N_, 1);
  ///参数增量
  Eigen::Matrix<double, Eigen::Dynamic, 1>  delta_x(param_N_, 1);
  Eigen::Matrix<double, Eigen::Dynamic, 1>  last_delta_x(param_N_, 1);

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> I(param_N_, param_N_);
  for(int i = 0;i<param_N_;i++){
    for(int j=0;j<param_N_;j++){
      if(i==j){
        I(i,j) = 1;
      }else{
        I(i,j) = 0;
      }
    }
  }

  ///LM 算法阻尼系数
  double lambda = 1;
  double v = 2;

  double last_cost_sum = std::numeric_limits<double>::max();
  while(!finished && iter_count++<max_iter_num){
    double cost_sum = EvaluateJFx(J, fx);

    if(iter_count > 1){
      ///L(0) - L(delta_x) =
      ///     0.5*delta_x.transpose() * ( _miu*delta_x - JtF)
      double L0_Ldelta =
          0.5*last_delta_x.transpose() * (lambda*last_delta_x - last_JtF);

      double rho = (last_cost_sum - cost_sum) / L0_Ldelta;
      if(rho > 0){
        //lambda := lambda ∗ max{ 1/3 , 1 − (2rho − 1)^3}; ν := 2
        lambda = lambda * std::max(double(1)/double(3) , (double(1)-std::pow((double(2) * rho-double(1)), 3)));
        v = 2;
      }else{
        //rho < 0 的情况应该舍去
        parameter_->BackToLastState();
        lambda *= v;
        v *= 2;
      }
      std::cout<<"lambda: "<<lambda<<std::endl;
//      std::cout<<"L0_Ldelta: "<<L0_Ldelta<<std::endl;
//      std::cout<<"last_cost_sum - cost_sum: "<<last_cost_sum - cost_sum<<std::endl;
//      std::cout<<"rho: "<<rho<<std::endl;
    }


    H = J.transpose() * J;
    _JtF = -J.transpose() * fx;
    last_JtF = -_JtF;

    delta_x = (H + lambda * I).colPivHouseholderQr().solve(_JtF);
    last_delta_x = delta_x;

    double delta_x_norm = delta_x.norm();

    parameter_->Plus(&delta_x[0]);

    if(delta_x_norm < 1e-10){
      finished = true;
    }

    last_cost_sum = cost_sum;
  }
}

double Solver::EvaluateJFx(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& J,
                         Eigen::Matrix<double, Eigen::Dynamic, 1>& fx){
  double cost_sum = 0;
  using MatrixType = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;
  using MapType = Eigen::Map<MatrixType>;
  int res_start = 0;
  for(auto & f : factors_){
    cost_sum += f->Evaluate();

    const int& res_n = f->GetResN();
    MapType f_infactor(f->GetResiduals(), res_n, 1);
    fx.block( res_start, 0, res_n,1) = MapType(f->GetResiduals(), res_n, 1);
    J.block( res_start, 0, res_n,param_N_) = MapType(f->GetJacobians(), res_n, param_N_);
    res_start += res_n;
  }

  return cost_sum;
}
