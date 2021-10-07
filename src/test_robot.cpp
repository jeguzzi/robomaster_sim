// main.cpp
#include <iostream>

#include "spdlog/spdlog.h"

#include "dummy_robot.hpp"

void test_gimbal(DummyRobot &robot) {
  // robot.gimbal.set_mode(Gimbal::Mode::velocity_mode);
  // robot.gimbal.set_mode(Gimbal::Mode::chassis_attitude_mode);
  robot.chassis.set_target_velocity({.theta = deg2rad(90)});
  // spdlog::info("Set gimbal target speed to 90 and 10 deg/s");
  // robot.gimbal.set_target_speeds({.yaw = deg2rad(90), .pitch = deg2rad(10)});
  //
  robot.move_gimbal(deg2rad(90), deg2rad(-10), deg2rad(10), deg2rad(10), Gimbal::Frame::fixed,
                    Gimbal::Frame::fixed);
  for (size_t i = 0; i < 40; i++) {
    robot.do_step(0.1);
    spdlog::info("******* {} *******", i);
    spdlog::info("Gimbal attitude:\n\tin world frame: {}\n\tin chassis frame: {}",
                 robot.gimbal.attitude(Gimbal::Frame::fixed),
                 robot.gimbal.attitude(Gimbal::Frame::chassis));
    spdlog::info("Gimbal angular velocity:\n\tin world frame: {}\n\tin chassis frame: {}",
                 robot.gimbal.angular_velocity(Gimbal::Frame::fixed),
                 robot.gimbal.angular_velocity(Gimbal::Frame::chassis));

    spdlog::info("Gimbal servo angles {}", robot.gimbal.current_angles());
    spdlog::info("Gimbal servo speeds {}", robot.gimbal.current_speeds());
    // spdlog::info("Gimbal servo target speeds {}", robot.gimbal.target_speeds());
  }
}

void test_LED(DummyRobot &robot) {
  spdlog::info("Switch LED to red", robot.play_sound(1, 1));
  robot.set_led_effect({.r = 255}, 0xff, 0x3, ActiveLED::LedEffect::on, 1.0, 1.0, true);
  for (size_t i = 0; i < 2; i++) {
    robot.do_step(0.1);
  }
  spdlog::info("Switch the first left and right gimbal LED to blue");
  robot.set_led_effect({.b = 255}, 0xff, 0x1, ActiveLED::LedEffect::on, 1.0, 1.0, true);
  for (size_t i = 0; i < 2; i++) {
    robot.do_step(0.1);
  }
}

void test_sound(DummyRobot &robot) {
  spdlog::info("Will play sound? {}", robot.play_sound(1, 1));
  for (size_t i = 0; i < 100; i++) {
    robot.do_step(0.1);
  }
}

void test_chassis(DummyRobot &robot) {
  spdlog::info("Will move? {}", robot.move_base({.x = 1.0f}, 1.0f, 1.0f));
  for (size_t i = 0; i < 100; i++) {
    robot.do_step(0.1);
  }
}

void test_arm(DummyRobot &robot) {
  spdlog::info("Will move arm? {}", robot.move_arm(0.2f, 0.0f, true));
  for (size_t i = 0; i < 100; i++) {
    robot.do_step(0.1);
  }
  return;
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
  spdlog::info("ARM {}", robot.arm.get_position());

  robot.set_target_servo_angle(0, 0.073304);
  for (size_t i = 0; i < 100; i++) {
    robot.do_step(0.1);
  }
  spdlog::info("LEFT {} {}", robot.get_servo_angle(0), robot.get_servo_speed(0));
  spdlog::info("RIGHT {} {}", robot.get_servo_angle(1), robot.get_servo_speed(1));
  spdlog::info("ARM {}", robot.arm.get_position());
}

void test_servo(DummyRobot &robot) {
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
    spdlog::info("ARM {}", robot.arm.get_position());
  }
}

int main(int argc, char **argv) {
  spdlog::info("Welcome to the robomaster simulation");
  char log_level[100] = "info";
  for (int i = 0; i < argc; i++) {
    if (sscanf(argv[i], "--log_level=%s", log_level)) {
      continue;
    }
  }
  spdlog::set_level(spdlog::level::from_str(log_level));
  DummyRobot robot(true, true, {true, true, false}, false, true, false);
  spdlog::info("Start run loop");

  // test_gimbal(robot);
  test_arm(robot);

  spdlog::info("Finished run loop");
  spdlog::info("Goodbye");
  return 0;
}
