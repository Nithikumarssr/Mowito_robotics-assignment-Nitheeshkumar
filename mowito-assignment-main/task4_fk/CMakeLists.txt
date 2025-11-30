cmake_minimum_required(VERSION 3.8)
project(forward_kinematics)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Eigen (for matrices/quaternions)
find_package(Eigen3 REQUIRED)

add_executable(forward_kinematics src/main.cpp)
target_link_libraries(forward_kinematics Eigen3::Eigen)
target_include_directories(forward_kinematics PRIVATE ${EIGEN3_INCLUDE_DIR})

# Optional: Install
install(TARGETS forward_kinematics DESTINATION bin)
