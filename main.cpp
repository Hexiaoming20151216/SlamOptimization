#include <iostream>
#include "math/so3.h"
#include "optimization/solver.h"

using namespace slamopt;
int main() {
  std::cout << "Hello, World!" << std::endl;
  Eigen::Quaterniond start_q(1,0,0,0);
  SO3d so_0(start_q);
  auto q_plus_delta = so_0.Plus(Eigen::Vector3d(0.1,0.2,-0.1));
  std::cout<<start_q.coeffs()<<std::endl;
  std::cout<<q_plus_delta.coeffs()<<std::endl;

  std::vector<Eigen::Vector3d> points;
  points.emplace_back(Eigen::Vector3d{1, 2, 3});
  points.emplace_back(Eigen::Vector3d{3, 2, -6});
  points.emplace_back(Eigen::Vector3d{6, -2, 9});
  points.emplace_back(Eigen::Vector3d{4, -4, -1});
  points.emplace_back(Eigen::Vector3d{-4, 14, 66});
  points.emplace_back(Eigen::Vector3d{-10, 33, -11});
  points.emplace_back(Eigen::Vector3d{-4, -9, 6});
  points.emplace_back(Eigen::Vector3d{-33, -19, -96});
  points.emplace_back(Eigen::Vector3d{-1, -4, 6});
  points.emplace_back(Eigen::Vector3d{33, 22, 6});
  points.emplace_back(Eigen::Vector3d{133, 222, -96});

  Solver solver;
  ///位姿参数: 旋转（四元数） + 平移，共七维
  std::shared_ptr<PoseParameter> pose_parameter =
      std::make_shared<PoseParameter>(q_plus_delta, Eigen::Vector3d{0.1,-0.1,0.2});
  solver.AddParams(pose_parameter);
  for(auto &pt: points){
    Factor::Ptr f = std::make_shared<PointPointFactor>(pt, pt);
    solver.AddFactor(f);
  }
  solver.run(11);

  auto parameters = pose_parameter->GetParams();
  std::cout<<"parameters_"<<std::endl;

  for(int i=0;i<7;i++){
    std::cout<<", "<<parameters[i];
  }

//
  Eigen::Map<Eigen::Quaterniond> rotation(parameters);
  Eigen::Map<Eigen::Vector3d> trans(parameters+4);

  ///将激光系下的点转到世界系下
  for(auto &pt: points){
    Eigen::Vector3d point0_w = rotation*pt + trans;
    std::cout<<"pt： "<<pt.x()<<", "<<pt.y()<<", "<<pt.z()<<std::endl;

  }

  return 0;
}