cmake_minimum_required(VERSION 3.16.3)
project(BasicRoboticsTechnologies)
add_compile_options(-std=c++17)

set(LOCALIZATION_SOURCE
  src/localization/ekf_localization.cpp
  src/localization/extended_kalman_filter.cpp
  src/localization/mobile_motion_model.cpp
  src/localization/gps_observation.cpp
)

set(PLANNING_SOURCE
  src/motion_planning/motion_planning.cpp
  src/motion_planning/astar.cpp
)

set(MANIPULATOR_SOURCE
  src/manipulator_control/controller.cpp
  src/manipulator_control/manipulator_2d.cpp
  src/manipulator_control/pid_controller.cpp
)

add_executable(ekf_localization ${LOCALIZATION_SOURCE})
add_executable(motion_planning ${PLANNING_SOURCE})
add_executable(controller ${MANIPULATOR_SOURCE})