#include <fstream>
#include <iostream>
#include <stdexcept>

#include "robot/robot.hpp"

// TODO(Jerome) [later] expose methods in lua to start an action (and enquire state)

#define CHECK_SERVO_LIMITS

Robot::Robot(bool _has_arm, bool _has_gripper, ServoValues<bool> _has_servo, bool _has_gimbal,
             bool _has_camera, bool _has_tof)

    : servos({Servo(_has_servo[0]), Servo(_has_servo[1]), Servo(_has_servo[2])})
    , arm(servos[0], servos[1])
    , has_arm(_has_arm && _has_servo[0] && _has_servo[1])
    , has_gripper(_has_gripper)
    , has_servo(_has_servo)
    , has_gimbal(_has_gimbal)
    , has_camera(_has_camera)
    , has_tof(_has_tof)
    , connected_servos()
    , mode(Mode::FREE)
    , sdk_enabled(false)
    , time_(0.0f)
    , callbacks()
    , actions() {
  for (size_t i = 0; i < has_servo.size(); i++) {
    if (has_servo[i])
      connected_servos[i] = &servos[i];
  }
  if (has_gimbal) {
    connected_servos[3] = &gimbal.yaw_servo;
    connected_servos[4] = &gimbal.pitch_servo;
  }
  std::string modules = "";
  if (has_arm)
    modules += "arm ";
  for (size_t i = 0; i < servos.size(); i++) {
    if (has_servo[i])
      modules += "servo#" + std::to_string(i) + " ";
  }
  if (has_gripper)
    modules += "gripper ";
  if (has_gimbal)
    modules += "gimbal ";
  if (has_camera)
    modules += "camera ";
  if (has_tof)
    modules += "ToF ";
  spdlog::info("Initialized a robot with these optional modules: {}", modules);
}

void Robot::read_chassis() {
  chassis.wheel_speeds.current = read_wheel_speeds();
  chassis.wheel_angles = read_wheel_angles();
  chassis.update_odometry(last_time_step);
  chassis.imu = read_imu();
  chassis.update_attitude(last_time_step);
}

void Robot::control_chassis() {
  if (chassis.wheel_motors_engaged && chassis.wheel_speeds.check()) {
    spdlog::debug("target_wheel_speed -> desired_target_wheel_speed = {}",
                  chassis.wheel_speeds.target);
    forward_target_wheel_speeds(chassis.wheel_speeds.target);
  }
}

void Robot::control_servos() {
#ifdef CHECK_SERVO_LIMITS
  if (has_arm) {
    arm.limit_motors(last_time_step);
  }
#endif
  for (auto const &[i, servo] : connected_servos) {
    if (servo->enabled.check()) {
      forward_servo_enabled(i, servo->enabled.target);
    }
    if (!servo->enabled.target) {
      continue;
    }
    if (servo->mode.check()) {
      forward_servo_mode(i, servo->mode.target);
    }
    if (servo->mode.target == Servo::SPEED) {
      float desired_speed = servo->desired_speed(last_time_step);
      if (servo->speed.target != desired_speed) {
        spdlog::debug("servo {}, set target speed to {}", i, desired_speed);
        servo->speed.target = desired_speed;
        forward_target_servo_speed(i, servo->speed.target);
      }
    } else {
      if (servo->angle.check()) {
        spdlog::debug("Servo {}, set target angle to {}", i, servo->angle.target);
        forward_target_servo_angle(i, servo->angle.target);
      }
    }
  }
}

void Robot::control_leds() {
  chassis_leds.do_step(last_time_step);
  for (size_t i = 0; i < chassis_leds.size; i++) {
    if (chassis_leds[i].check()) {
      spdlog::debug("chassis led {} -> {}", i, chassis_leds[i].color);
      forward_chassis_led(i, chassis_leds[i].color);
    }
  }
  if (has_gimbal) {
    // spdlog::info("update gimbal leds");
    gimbal_leds.do_step(last_time_step);
    for (size_t i = 0; i < gimbal_leds.size; i++) {
      for (size_t j = 0; j < gimbal_leds[i].number; j++) {
        if (gimbal_leds[i].check(j)) {
          spdlog::debug("gimbal led {} {} -> {}", i, j, gimbal_leds[i].get_color(j));
          forward_gimbal_led(i, j, gimbal_leds[i].get_color(j));
        }
      }
    }
    if (blaster_led.intensity.check()) {
      forward_blaster_led(blaster_led.intensity.target);
    }
  }
}

void Robot::do_step(float time_step) {
  time_ += time_step;
  last_time_step = time_step;

  read_chassis();
  read_hit_events();
  read_ir_events();
  if (has_tof) {
    size_t index = 0;
    for (auto & tof_reading : tof.readings) {
      if (tof_reading.active) {
        tof_reading.distance = read_tof(index);
      }
      index++;
    }
  }
  if (has_gripper) {
    gripper.state.current = read_gripper_state();
  }
  if (has_camera) {
    if (vision.enabled) {
      vision.detected_objects = read_detected_objects();
    }
    // Stream the camera image
    if (camera.streaming) {
      spdlog::debug("[Robot] capture new camera frame");
      camera.image = read_camera_image();
      // std::cout << int(image[0]) << " " << int(image[1]) << " " << int(image[2]) << " (" <<
      // image.size() <<")\n"; auto image = std::vector<unsigned char>(640 * 360 * 3, 0); auto image
      // = _image; static unsigned seq = 0; seq++; std::string name =
      // "/Users/Jerome/Dev/My/robomaster_simulation/build/images/image" + std::to_string(seq) +
      // ".dat"; std::ofstream fout(name.c_str(), std::ios::out | std::ios::binary);
      // fout.write((char
      // *)image.data(), image.size()); fout.close();
    }
  }

  for (auto const &[i, servo] : connected_servos) {
    servo->angle.current = read_servo_angle(i);
    servo->speed.current = read_servo_speed(i);
  }

  if (has_arm) {
    arm.update_position();
  }

  // Run Actions. Using iterators to safely delete a key.
  auto it = actions.cbegin();
  while (it != actions.cend()) {
    it->second->do_step_cb(time_step);
    if (it->second->done()) {
      previous_action_state[it->first] = it->second->state;
      it = actions.erase(it);
    } else {
      it++;
    }
  }

  if (has_gimbal) {
    gimbal.update_control(time_step, chassis.attitude, chassis.imu);
  }

  control_chassis();
  control_leds();
  // DONE(Jerome): complete Gimbal. Can we use the same interface as servos?
  control_servos();

  // Gripper
  if (has_gripper) {
    if (gripper.state.check() || gripper.power.check()) {
      forward_target_gripper(gripper.state.target, gripper.power.target);
    }
  }

  // Update Publishers

  for (auto &cb : callbacks)
    cb(time_step);
}

void Robot::set_led_effect(Color color, LedMask mask, CompositeLedMask led_mask,
                           ActiveLED::LedEffect effect, float period_on, float period_off,
                           bool loop) {
  spdlog::info("[Robot] set led effect: color {}, mask {}, led_mask {}, effect {}", color, mask,
               led_mask, effect);
  if (mask & ARMOR_BOTTOM_BACK)
    chassis_leds.rear.update(color, effect, period_on, period_off, loop);
  if (mask & ARMOR_BOTTOM_FRONT)
    chassis_leds.front.update(color, effect, period_on, period_off, loop);
  if (mask & ARMOR_BOTTOM_LEFT)
    chassis_leds.left.update(color, effect, period_on, period_off, loop);
  if (mask & ARMOR_BOTTOM_RIGHT)
    chassis_leds.right.update(color, effect, period_on, period_off, loop);
  if (mask & ARMOR_TOP_LEFT)
    gimbal_leds.top_left.update(color, effect, period_on, period_off, loop, led_mask);
  if (mask & ARMOR_TOP_RIGHT)
    gimbal_leds.top_right.update(color, effect, period_on, period_off, loop, led_mask);
}

void Robot::set_mode(Mode _mode) {
  spdlog::info("[Robot] Set mode {}", _mode);
  mode = _mode;
  gimbal.follow_chassis(mode == CHASSIS_LEAD);
}

Robot::Mode Robot::get_mode() { return mode; }

void Robot::set_enable_sdk(bool value) {
  if (sdk_enabled == value)
    return;
  sdk_enabled = value;
  if (value)
    spdlog::info("[Robot] enabled SDK");
  else
    spdlog::info("[Robot] disabled SDK");
}

bool Robot::start_streaming(unsigned width, unsigned height) {
  if (!last_time_step) {
    spdlog::warn("[Robot] time step unknown: cannot set video streamer fps");
    return false;
  }
  camera.fps = 1.0f / last_time_step;
  if (!forward_camera_resolution(width, height)) {
    spdlog::warn("[Robot] Camera resolution {} x {} not supported", width, height);
    return false;
  }
  camera.width = width;
  camera.height = height;
  camera.streaming = true;
  return true;
}

bool Robot::stop_streaming() {
  spdlog::info("[Robot] stop streaming");
  camera.streaming = false;
  return true;
}

Action::State Robot::move_base(const Pose2D &pose, float linear_speed, float angular_speed) {
  auto a = std::make_unique<MoveAction>(this, pose, linear_speed, angular_speed);
  return submit_action(std::move(a));
}

Action::State Robot::move_arm(float x, float z, bool absolute) {
  auto a = std::make_unique<MoveArmAction>(this, x, z, absolute);
  return submit_action(std::move(a));
}

Action::State Robot::play_sound(unsigned sound_id, unsigned times) {
  auto a = std::make_unique<PlaySoundAction>(this, sound_id, times);
  return submit_action(std::move(a));
}

Action::State Robot::move_servo(size_t id, float target_angle) {
  auto a = std::make_unique<MoveServoAction>(this, id, target_angle);
  return submit_action(std::move(a));
}

Action::State Robot::move_gimbal(float target_yaw, float target_pitch, float yaw_speed,
                                 float pitch_speed, Gimbal::Frame yaw_frame,
                                 Gimbal::Frame pitch_frame) {
  auto a = std::make_unique<MoveGimbalAction>(this, target_yaw, target_pitch, yaw_speed,
                                              pitch_speed, yaw_frame, pitch_frame);
  return submit_action(std::move(a));
}

Action::State Robot::submit_action(std::unique_ptr<MoveAction> action) {
  // TODO(Jerome): What should we do if an action is already active?
  if (actions.count("move"))
    return Action::State::rejected;
  // actions["move"] = std::move(action);
  action->state = Action::State::started;
  actions.emplace("move", std::move(action));
  spdlog::info("Start new MoveAction");
  return actions["move"]->state;
}

Action::State Robot::submit_action(std::unique_ptr<MoveArmAction> action) {
  if (!has_arm) {
    // TODO(Jerome): check what happens with real robots
    spdlog::warn("Arm not connected: cannot do action");
    return Action::State::rejected;
  }
  // TODO(Jerome): What should we do if an action is already active?
  if (actions.count("move_arm"))
    return Action::State::rejected;
  // actions["move_arm"] = action;
  action->state = Action::State::started;
  actions.emplace("move_arm", std::move(action));
  spdlog::info("Start new MoveArmAction");
  return actions["move_arm"]->state;
}

Action::State Robot::submit_action(std::unique_ptr<PlaySoundAction> action) {
  // TODO(Jerome): What should we do if an action is already active?
  if (actions.count("play_sound"))
    return Action::State::rejected;
  // actions["play_sound"] = action;
  action->state = Action::State::started;
  actions.emplace("play_sound", std::move(action));
  spdlog::info("Start new PlaySoundAction");
  return actions["play_sound"]->state;
}

Action::State Robot::submit_action(std::unique_ptr<MoveServoAction> action) {
  // TODO(Jerome): What should we do if an action is already active?
  if (action->servo_id > has_servo.size() || !has_servo[action->servo_id]) {
    // TODO(Jerome): check what happens with real robots
    spdlog::warn("Servo {} not connected: cannot do action", action->servo_id);
    return Action::State::rejected;
  }
  if (actions.count("move_servo"))
    return Action::State::rejected;
  // actions["play_sound"] = action;
  action->state = Action::State::started;
  actions.emplace("move_servo", std::move(action));
  spdlog::info("Start new MoveServoAction");
  return actions["move_servo"]->state;
}

Action::State Robot::submit_action(std::unique_ptr<MoveGimbalAction> action) {
  if (!has_gimbal) {
    // TODO(Jerome): check what happens with real robots
    spdlog::warn("Gimbal not connected: cannot do action");
    return Action::State::rejected;
  }
  // TODO(Jerome): What should we do if an action is already active?
  if (actions.count("move_gimbal"))
    return Action::State::rejected;
  // actions["move"] = std::move(action);
  action->state = Action::State::started;
  actions.emplace("move_gimbal", std::move(action));
  spdlog::info("Start new MoveGimbalAction");
  return actions["move_gimbal"]->state;
}
