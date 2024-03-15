cmake_minimum_required(VERSION 3.16.3)
project(localization CXX)

add_compile_options(-std=c++17)

#find_package(Eigen3 REQUIRED)
include_directories(
  include
)

add_library(extended_kalman_filter src/lib/filter/extended_kalman_filter.cpp)
add_library(motion_model src/lib/model/mobile_motion_model.cpp)
add_library(observation_model src/lib/model/gps_observation.cpp)

add_executable(ekf_localization src/ekf_localization.cpp)
target_link_libraries(ekf_localization extended_kalman_filter motion_model observation_model)