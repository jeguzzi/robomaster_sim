cmake_minimum_required(VERSION 3.1...3.19)

project(
  robomaster_simulation
  VERSION 0.1
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)

add_library(robomaster robomaster.cpp)
add_executable(test test.cpp)

target_link_libraries(test PRIVATE robomaster)
