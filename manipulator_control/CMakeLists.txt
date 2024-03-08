cmake_minimum_required(VERSION 3.10)
project(manipulator_control CXX)

add_compile_options(-std=c++17)

include_directories(include)

add_library(manipulator_2d src/lib/manipulator/manipulator_2d.cpp)
add_library(pid_controller src/lib/controller/pid_controller.cpp)

add_executable(controller src/controller.cpp)
target_link_libraries(controller manipulator_2d pid_controller)
