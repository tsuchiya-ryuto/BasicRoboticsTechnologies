cmake_minimum_required(VERSION 3.10)
project(optimization CXX)

add_compile_options(-std=c++17)

include_directories(include)

add_executable(least_square src/least_square.cpp)