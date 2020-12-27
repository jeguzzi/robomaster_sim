cmake_minimum_required(VERSION 3.1...3.19)

project(
  robomaster_simulation
  VERSION 0.1
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

add_compile_options(-Wall -Wextra -pedantic -Werror -Wno-unused-parameter)

find_package(Boost COMPONENTS system REQUIRED)
find_package(spdlog REQUIRED)

# link_directories(${Boost_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIRS} include)

add_library(robomaster
  src/robomaster.cpp
  src/server.cpp
  src/command.cpp
  src/connection.cpp
  src/topics.cpp
)
add_executable(test test.cpp)

target_link_libraries(test PRIVATE
  robomaster
  PRIVATE spdlog::spdlog
  # PRIVATE spdlog::spdlog_header_only
  # ${Boost_LIBRARIES}
)
