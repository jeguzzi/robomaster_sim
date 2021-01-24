// main.cpp
#include <iostream>

#include "spdlog/spdlog.h"

#include "dummy_robot.hpp"

int main(int argc, char **argv) {
  spdlog::info("Welcome to the robomaster simulation");
  char log_level[100] = "info";
  for (int i = 0; i < argc; i++) {
    if (sscanf(argv[i], "--log_level=%s", log_level)) {
      continue;
    }
  }
  spdlog::set_level(spdlog::level::from_str(log_level));
  DummyRobot robot;
  spdlog::info("Start run loop");
  spdlog::info("Will play sound? {}", robot.play_sound(1, 1));
  spdlog::info("Will move? {}", robot.move({.x = 1.0f}, 1.0f, 1.0f));
  spdlog::info("Will move arm? {}", robot.move_arm(0.1f, 0.0f, true));
  for (size_t i = 0; i < 100; i++) {
    robot.do_step(0.1);
  }
  spdlog::info("Finished run loop");
  spdlog::info("Goodbye");
  return 0;
}
