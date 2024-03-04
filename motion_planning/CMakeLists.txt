cmake_minimum_required(VERSION 3.10)
project(motion_planning CXX)

add_compile_options(-std=c++17)

include_directories(include)

add_executable(astar src/astar.cpp)