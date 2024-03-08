cmake_minimum_required(VERSION 3.10)
project(motion_planning CXX)

add_compile_options(-std=c++17)

include_directories(include)

add_library(astar src/lib/astar.cpp)

add_executable(motion_planning src/motion_planning.cpp)
target_link_libraries(motion_planning astar)