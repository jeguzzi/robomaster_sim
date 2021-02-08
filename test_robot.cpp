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
  spdlog::info("Will move? {}", robot.move_base({.x = 1.0f}, 1.0f, 1.0f));
  for (size_t i = 0; i < 100; i++) {
    robot.do_step(0.1);
  }
  spdlog::info("Will move arm? {}", robot.move_arm(0.1f, 0.0f, true));
  for (size_t i = 0; i < 100; i++) {
    robot.do_step(0.1);
  }
  float target_angle = 1.2;
  spdlog::info("Will move servo from {} to {}? {}", robot.get_servo_angle(1), target_angle,
               robot.move_servo(1, target_angle));
  for (size_t i = 0; i < 100; i++) {
    robot.do_step(0.1);
  }

  spdlog::info("Will move arm? {}", robot.move_arm(0.0832229, 0.157686, true));
  for (size_t i = 0; i < 100; i++) {
    robot.do_step(0.1);
  }
  spdlog::info("LEFT {} {}", robot.get_servo_angle(0), robot.get_servo_speed(0));
  spdlog::info("RIGHT {} {}", robot.get_servo_angle(1), robot.get_servo_speed(1));
  spdlog::info("ARM {}", robot.get_arm_position());

  // robot.set_target_servo_angle(0, 0.073304);
  // for (size_t i = 0; i < 100; i++) {
  //   robot.do_step(0.1);
  // }
  // spdlog::info("LEFT {} {}", robot.get_servo_angle(0), robot.get_servo_speed(0));
  // spdlog::info("RIGHT {} {}", robot.get_servo_angle(1), robot.get_servo_speed(1));
  // spdlog::info("ARM {}", robot.get_arm_position());

  robot.set_target_servo_speed(1, 1);
  // robot.set_target_servo_speed(0, 1);
  // robot.set_target_servo_angle(1, 0);
  robot.set_servo_mode(1, Servo::SPEED);
  // robot.set_servo_mode(0, Servo::SPEED);
  robot.enable_servo(1, true);
  // robot.enable_servo(0, true);
  for (size_t i = 0; i < 20; i++) {
    robot.do_step(0.1);
    spdlog::info("LEFT {} {}", robot.get_servo_angle(0), robot.get_servo_speed(0));
    spdlog::info("RIGHT {} {}", robot.get_servo_angle(1), robot.get_servo_speed(1));
    spdlog::info("ARM {}", robot.get_arm_position());
  }

  spdlog::info("Finished run loop");
  spdlog::info("Goodbye");
  return 0;
}
