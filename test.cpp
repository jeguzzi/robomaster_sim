// main.cpp
#include <iostream>
#include "robomaster.hpp"
#include "dummy_robot.hpp"

// #include "spdlog/cfg/env.h"


int main(int argc, char **argv) {
  std::cout << "Welcome to the robomaster simulation" << std::endl;
  bool use_udp = false;
  long bitrate = 200000;
  char serial[100] = "RM0001";
  char log_level[100] = "info";
  for (int i = 0; i < argc; i++) {
    if(strcmp(argv[i], "--udp") == 0) {
      use_udp = true;
      continue;
    }
    if(sscanf(argv[i],"--bitrate=%ld", &bitrate)) {
      continue;
    }
    if(sscanf(argv[i],"--serial_number=%s", serial)) {
      continue;
    }
    if(sscanf(argv[i],"--serial_number=%s", serial)) {
      continue;
    }
    if(sscanf(argv[i],"--log_level=%s", log_level)) {
      continue;
    }
  }
  // TODO(spdlog bug?): not working here vs in robomaster constructor whyyyy?
  // spdlog::set_level(spdlog::level::debug);
  // spdlog::cfg::load_env_levels();
  std::cout << "LOG_LEVEL " << spdlog::level::from_str(log_level) << std::endl;
  boost::asio::io_context io_context;
  spdlog::info("Welcome to the RoboMaster test");
  // Get ignored
  spdlog::set_level(spdlog::level::from_str(log_level));
  DummyRobot dummy(&io_context, 0.05);
  rm::RoboMaster robot(&io_context, &dummy, std::string(serial), use_udp, bitrate);
  spdlog::info("Start spinning");
  robot.spin(false);
  std::cout << "Goodbye" << std::endl;
  return 0;
};
