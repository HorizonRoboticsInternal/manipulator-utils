cmake_minimum_required(VERSION 3.0)
project(kincpp)

set(CMAKE_CXX_FLAGS "-O3 -DNDEBUG -Wno-deprecated-declarations")
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-march=native" COMPILER_SUPPORTS_MARCH_NATIVE)
if(COMPILER_SUPPORTS_MARCH_NATIVE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
endif()

set(LIBRARY_OUTPUT_PATH "../kinematics_cpp")

find_package(pybind11 REQUIRED)
find_package(Eigen3 REQUIRED)

pybind11_add_module(kincpp kinematics_cpp/kinematics.cpp)

target_link_libraries(kincpp PRIVATE
    Eigen3::Eigen)
