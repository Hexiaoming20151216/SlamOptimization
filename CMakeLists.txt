cmake_minimum_required(VERSION 3.15)
project(slam_optimization)

set(CMAKE_CXX_STANDARD 14)
# Eigen
find_package(Eigen3 REQUIRED)
include_directories({EIGEN3_INCLUDE_DIRS})

add_executable(slam_optimization
        main.cpp
        optimization/solver.cpp
        )