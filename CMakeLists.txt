cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

add_compile_options(-std=c++11)

project(box_detect VERSION 0.1.0 LANGUAGES C CXX)
include_directories( "EIGEN_SEC/" )
add_subdirectory(src)  
