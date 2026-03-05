cmake_minimum_required(VERSION 3.1...3.19)

project(
  robomaster_simulation
  VERSION 1.3
  LANGUAGES CXX)

if(WIN32)
  set(CMAKE_CXX_STANDARD 20)
  add_compile_options(-fPIC -Wall -D_USE_MATH_DEFINES)
else()
  set(CMAKE_CXX_STANDARD 17)
  add_compile_options(
    -fPIC
    -Wall
    -Wextra
    -Werror
    -Wno-unused-parameter
    -Wno-deprecated-declarations
    -Wno-missing-field-initializers
    -Wno-unused-variable) # -Wno-c99-extensions
endif()

option(USE_OPENCV "use OpenCV for vision" OFF)
option(BUILD_COPPELIASIM_PLUGIN "build the CoppeliaSim plugin" ON)

set(CMAKE_BUILD_TYPE Release)
set(VCPKG_BUILD_TYPE release)

find_package(PkgConfig REQUIRED)
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()
find_package(Boost REQUIRED)
find_package(
  Boost REQUIRED
  COMPONENTS date_time thread
  OPTIONAL_COMPONENTS asio format)

find_package(spdlog CONFIG REQUIRED)
pkg_check_modules(AVCODEC REQUIRED IMPORTED_TARGET libavcodec)
pkg_check_modules(AVUTIL REQUIRED IMPORTED_TARGET libavutil)
pkg_check_modules(AVDEVICE REQUIRED IMPORTED_TARGET libavdevice)
pkg_check_modules(AVFORMAT REQUIRED IMPORTED_TARGET libavformat)

include_directories(include)

add_library(
  robot STATIC
  src/robot/action.cpp
  src/robot/arm.cpp
  src/robot/chassis.cpp
  src/robot/led.cpp
  src/robot/gimbal.cpp
  src/robot/robot.cpp
  src/robot/vision.cpp)

target_link_libraries(robot PUBLIC spdlog::spdlog)

add_executable(test_robot src/test_robot.cpp src/dummy_robot.cpp)

target_link_libraries(test_robot PRIVATE robot)

add_library(
  robomaster STATIC
  src/robot/action.cpp
  src/robot/arm.cpp
  src/robot/chassis.cpp
  src/robot/led.cpp
  src/robot/gimbal.cpp
  src/robot/robot.cpp
  src/robot/vision.cpp
  src/robomaster.cpp
  src/server.cpp
  src/command.cpp
  src/connection.cpp
  src/dummy_robot.cpp
  src/rt_dummy_robot.cpp
  src/protocol.cpp
  src/topic.cpp
  src/encoder.cpp
  src/streamer.cpp
  src/discovery.cpp)

target_link_libraries(
  robomaster
  PUBLIC PkgConfig::AVCODEC
         PkgConfig::AVFORMAT
         PkgConfig::AVUTIL
         PkgConfig::AVDEVICE
         spdlog::spdlog
         Boost::thread
         Boost::date_time)

if(TARGET Boost::asio)
  target_link_libraries(robomaster PUBLIC Boost::asio)
endif()
if(TARGET Boost::assign)
  target_link_libraries(robomaster PUBLIC Boost::assign)
endif()
if(TARGET Boost::format)
  target_link_libraries(robomaster PUBLIC Boost::format)
endif()
if(NOT
   ((TARGET Boost::asio)
    OR (TARGET Boost::assign)
    OR (TARGET Boost::format)))
  include_directories(${Boost_INCLUDE_DIRS})
endif()

if(CMAKE_COMPILER_IS_GNUCXX)
  target_link_libraries(robomaster PRIVATE "-Wl,-Bsymbolic")
endif()

add_executable(test src/test.cpp)

add_executable(test_encoder src/test_encoder.cpp src/encoder.cpp)

target_link_libraries(test PRIVATE robomaster)

target_link_libraries(
  test_encoder PRIVATE PkgConfig::AVCODEC PkgConfig::AVFORMAT PkgConfig::AVUTIL
                       PkgConfig::AVDEVICE spdlog::spdlog)

if(BUILD_COPPELIASIM_PLUGIN)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/coppeliaSim_plugin")

    if(WIN32)
      add_compile_options(-DSIM_REQUIRED_PROGRAM_VERSION_NB=4010000)
    endif()

    if(USE_OPENCV)
      find_package(
        OpenCV 4
        COMPONENTS core imgproc imgcodecs
        REQUIRED)
      add_compile_definitions(USE_OPENCV)
      message("Will use OpenCV for vision")
    endif()

    add_subdirectory(coppeliaSim_plugin)

    target_include_directories(${_PLUGIN_NAME} PRIVATE ${OpenCV_INCLUDE_DIRS})

    if(${COPPELIASIM_VERSION} VERSION_GREATER_EQUAL 4.6)
      set(KEYWORD PRIVATE)
    endif()

    target_link_libraries(${_PLUGIN_NAME} ${KEYWORD} robomaster)

    if(USE_OPENCV)
      target_link_libraries(${_PLUGIN_NAME} ${KEYWORD} ${OpenCV_LIBS})
    endif()

    if(WIN32)
      target_compile_options(${_PLUGIN_NAME} PUBLIC)
    else()
      target_compile_options(
        ${_PLUGIN_NAME}
        PUBLIC "-Wno-reorder-ctor"
               "-Wno-sign-compare"
               "-Wno-delete-non-abstract-non-virtual-dtor"
               "-Wno-unused-function"
               "-Wno-reorder"
               "-Wno-delete-non-virtual-dtor"
               "-Wno-unused-result")
    endif()
  endif()
endif()
