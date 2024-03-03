cmake_minimum_required(VERSION 3.16.3)
project(localization CXX)

add_compile_options(-std=c++17)

#find_package(Eigen3 REQUIRED)
include_directories(
  include
)

add_executable(ekf_localization src/ekf_localization.cpp)