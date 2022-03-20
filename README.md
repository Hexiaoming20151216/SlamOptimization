# SlamOptimization
本工程基于Eigen库实现了流形上的Levenberg-Marquardt非线性优化算法的 demo 框架

求解器、参数块、残差因子分别为独立模块，对于任何非线性最小二乘问题，只需基于parameter和factor基类，自定义参数和残差更新方式和雅克比求解方法，即可自动求解。

main函数中的例子实现了 point to point的icp方法, 其中基于Eigen实现了位姿（旋转+平移）的Manifold和tangent之间的指数与对数映射方法

#**编译与运行**

本工程只依赖于Eigen库， c++版本 >= 11

`mkdir build && cd build && cmake ..`

`make`

`./slam_optimization`