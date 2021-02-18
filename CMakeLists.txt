cmake_minimum_required(VERSION 3.1...3.19)

project(
  robomaster_simulation
  VERSION 0.1
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

add_compile_options(-fPIC -Wall -Wextra  -Werror -Wno-unused-parameter -Wno-deprecated-declarations -Wno-c99-extensions)
# -pedantic
find_package(Boost COMPONENTS system thread REQUIRED)
find_package(spdlog REQUIRED )
# find_package(libav REQUIRED avcodec REQUIRED)

# if(libav)
#   message("Found FFMPEG/LibAV libraries")
#   include_directories(${LIBAV_INCLUDE_DIR})
# else()
#   message("Can't find libavcodec, libavformat or libavutil. Add them!")
# endif()
#

find_path(AVCODEC_INCLUDE_DIR libavcodec/avcodec.h)
find_library(AVCODEC_LIBRARY avcodec)

find_path(AVFORMAT_INCLUDE_DIR libavformat/avformat.h)
find_library(AVFORMAT_LIBRARY avformat)

find_path(AVUTIL_INCLUDE_DIR libavutil/avutil.h)
find_library(AVUTIL_LIBRARY avutil)

find_path(AVDEVICE_INCLUDE_DIR libavdevice/avdevice.h)
find_library(AVDEVICE_LIBRARY avdevice)


# link_directories(${Boost_LIBRARY_DIRS})
include_directories(
  ${Boost_INCLUDE_DIRS}
  ${AVCODEC_INCLUDE_DIR}
  ${AVFORMAT_INCLUDE_DIR}
  ${AVUTIL_INCLUDE_DIR}
  ${AVDEVICE_INCLUDE_DIR}
  ${spdlog_INCLUDE_DIR}
  include
)

add_library(robot SHARED
  src/robot
)

target_link_libraries(robot PRIVATE
  spdlog::spdlog
)

add_executable(test_robot test_robot.cpp src/dummy_robot.cpp)

target_link_libraries(test_robot PRIVATE
  robot
  spdlog::spdlog
)

add_library(robomaster SHARED
  src/robomaster.cpp
  src/server.cpp
  src/command.cpp
  src/connection.cpp
  src/dummy_robot.cpp
  src/rt_dummy_robot.cpp
  src/protocol.cpp
  src/topic.cpp
  # src/rt_topic.cpp
  src/robot.cpp
  # src/action.cpp
  # src/utils.cpp
  src/encoder.cpp
  src/streamer.cpp
  src/discovery.cpp
)

target_link_libraries(robomaster PRIVATE
  spdlog::spdlog
  ${AVCODEC_LIBRARY}
  ${AVFORMAT_LIBRARY}
  ${AVUTIL_LIBRARY}
  ${AVDEVICE_LIBRARY}
  ${Boost_LIBRARIES}
)

add_executable(test test.cpp)

add_executable(test_encoder test_encoder.cpp src/encoder.cpp)

target_link_libraries(test PRIVATE
  robomaster
  spdlog::spdlog
  ${Boost_LIBRARIES}
)

# message("${AVCODEC_LIBRARY} ${AVFORMAT_LIBRARY} ${AVUTIL_LIBRARY} ${AVDEVICE_LIBRARY}")

target_link_libraries(test_encoder PRIVATE
  ${AVCODEC_LIBRARY}
  ${AVFORMAT_LIBRARY}
  ${AVUTIL_LIBRARY}
  ${AVDEVICE_LIBRARY}
  spdlog::spdlog
)

link_directories(
/usr/local/opt/icu4c/lib
)

add_subdirectory(coppeliaSim_plugin)

target_link_libraries(simExtRobomaster
  robomaster
  spdlog::spdlog
  ${Boost_LIBRARIES}
)

# set_target_properties( PROPERTIES COMPILE_FLAGS "Wno-reorder-ctor")
target_compile_options(simExtRobomaster PUBLIC
"-Wno-reorder-ctor"
"-Wno-sign-compare"
"-Wno-delete-non-abstract-non-virtual-dtor"
"-Wno-unused-function"
"-Wno-reorder"
"-Wno-delete-non-virtual-dtor"
"-Wno-unused-result"
)

# add_executable(test_log test_log.cpp)
#
# target_link_libraries(test_log PRIVATE
#   spdlog::spdlog
# )

# add_executable(test_tcp test_acceptor.cpp)
#
# target_link_libraries(test_tcp PRIVATE
#   ${Boost_LIBRARIES}
# )
