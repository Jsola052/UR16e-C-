cmake_minimum_required(VERSION 3.0.2)

project(
  ur16_move
  VERSION 1.0
  LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 20)

find_package(ur_rtde REQUIRED)
find_package(Eigen3 3.3 REQUIRED)

add_executable(
    ${PROJECT_NAME}
    grind.cpp
    UrRobot.cpp
    UrRobot.hpp)

target_link_libraries(
    ${PROJECT_NAME} 
    ur_rtde::rtde)

target_link_libraries(
  ${PROJECT_NAME} 
  Eigen3::Eigen)



