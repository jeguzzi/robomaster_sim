#include <iostream>

#include "robomaster.hpp"
#include "rt_dummy_robot.hpp"

// #include "spdlog/cfg/env.h"

int main(int argc, char **argv) {
  std::cout << "Welcome to the robomaster simulation" << std::endl;
  bool use_udp = false;
  unsigned bitrate = 200000;
  char serial[100] = "RM0001";
  char log_level[100] = "info";
  char ip[100] = "";
  for (int i = 0; i < argc; i++) {
    if (strcmp(argv[i], "--udp") == 0) {
      use_udp = true;
      continue;
    }
    if (sscanf(argv[i], "--bitrate=%d", &bitrate)) {
      continue;
    }
    if (sscanf(argv[i], "--serial_number=%s", serial)) {
      continue;
    }
    if (sscanf(argv[i], "--ip=%s", ip)) {
      continue;
    }
    if (sscanf(argv[i], "--log_level=%s", log_level)) {
      continue;
    }
  }
  // TODO(spdlog bug?): not working here vs in robomaster constructor whyyyy?
  // spdlog::set_level(spdlog::level::debug);
  // spdlog::cfg::load_env_levels();
  std::cout << "LOG_LEVEL " << spdlog::level::from_str(log_level) << std::endl;
  auto io_context = std::make_shared<boost::asio::io_context>();
  spdlog::info("Welcome to the RoboMaster test");
  // Get ignored
  spdlog::set_level(spdlog::level::from_str(log_level));
  RealTimeDummyRobot dummy(io_context.get(), 0.05);
  RoboMaster robot(io_context, &dummy, std::string(serial), use_udp, bitrate, ip);
  spdlog::info("Start spinning");
  robot.spin(false);
  std::cout << "Goodbye" << std::endl;
  return 0;
}
