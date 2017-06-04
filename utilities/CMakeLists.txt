cmake_minimum_required(VERSION 3.7)
project(utilities)

set(CMAKE_CXX_STANDARD 14)

set(SOURCE_FILES sph2sdf.cpp)
add_executable(sph2sdf ${SOURCE_FILES})