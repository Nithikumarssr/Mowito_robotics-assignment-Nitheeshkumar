cmake_minimum_required(VERSION 3.8)
project(euler_quat)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)

add_executable(euler_to_quat src/main.cpp)
target_link_libraries(euler_to_quat Eigen3::Eigen)

install(TARGETS euler_to_quat DESTINATION bin)

