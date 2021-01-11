// main.cpp
#include <iostream>
#include "robomaster.hpp"
#include "dummy_robot.hpp"

// #include "spdlog/cfg/env.h"


int main(int argc, char **argv) {
  std::cout << "Welcome to the robomaster simulation" << std::endl;
  // TODO(spdlog bug?): not working here vs in robomaster constructor whyyyy?
  // spdlog::set_level(spdlog::level::debug);
  // spdlog::cfg::load_env_levels();
  boost::asio::io_context io_context;
  DummyRobot dummy(&io_context, 0.05);
  rm::RoboMaster robot(&io_context, &dummy);
  robot.spin(false);
  std::cout << "Goodbye" << std::endl;
  return 0;
};
