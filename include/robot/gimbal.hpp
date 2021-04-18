#ifndef INCLUDE_ROBOT_GIMBAL_HPP_
#define INCLUDE_ROBOT_GIMBAL_HPP_

#include <algorithm>

#include "../utils.hpp"
#include "servo.hpp"

struct Gimbal {
  static constexpr float max_speed = deg2rad(540);

  enum Mode { attitude_mode, velocity_mode };

  enum Frame { chassis, fixed, gimbal };

  Servo yaw_servo;
  Servo pitch_servo;
  Mode mode;
  IMU chassis_imu;
  Attitude chassis_attitude;
  // The target angular speed in the frame `gimbal_fixed`: gravity aligned,
  // orientation fixed when the robot starts.
  Vector3 target_angular_velocity_in_gimbal_fixed;
  Attitude target_attitude;

  bool performing_action;
  bool following_chassis;

  GimbalValues<float> control_speed;

  void update_control(float time_step, Attitude attitude, IMU imu);

  Gimbal()
      : yaw_servo(true, deg2rad(-250), deg2rad(250), max_speed)
      , pitch_servo(true, deg2rad(-30), deg2rad(25), max_speed)
      , mode(Mode::attitude_mode)
      , chassis_imu()
      , chassis_attitude()
      , target_angular_velocity_in_gimbal_fixed()
      , target_attitude()
      , performing_action(false)
      , following_chassis(false) {}
  // float distance() const { return std::max(yaw_servo.distance(), pitch_servo.distance()); }

  float distance() const {
    if (mode == Mode::attitude_mode) {
      Attitude current;
      if (!performing_action && following_chassis) {
        current = attitude(Frame::chassis);
      } else {
        current = attitude(Frame::fixed);
      }
      return std::max(std::abs(current.yaw - target_attitude.yaw),
                      std::abs(current.pitch - target_attitude.pitch));
    }
    return 0.0;
  }
  // Angles in fixed frame only!!!
  void set_target_angles(GimbalValues<float> angle) {
    set_mode(Mode::attitude_mode);
    target_attitude = {.yaw = angle.yaw, .pitch = angle.pitch};
    // if (mode == Mode::attitude_mode) {
    //   target_attitude = target_attitude + chassis_attitude;
    // }
  }

  void set_target_speeds(GimbalValues<float> speed) {
    set_mode(Mode::velocity_mode);
    target_angular_velocity_in_gimbal_fixed = {.y = std::clamp(speed.pitch, -max_speed, max_speed),
                                               .z = std::clamp(speed.yaw, -max_speed, max_speed)};
  }

  void reset_target_speeds() { target_angular_velocity_in_gimbal_fixed = {}; }

  void set_control_speeds(GimbalValues<float> speed) {
    control_speed.pitch = std::clamp(speed.pitch, -max_speed, max_speed);
    control_speed.yaw = std::clamp(speed.yaw, -max_speed, max_speed);
  }

  Attitude attitude(Frame frame) const {
    Attitude value = {.yaw = yaw_servo.angle.current, .pitch = pitch_servo.angle.current};
    if (frame == Frame::fixed)
      value = value + chassis_attitude;
    return value;
  }

  GimbalValues<float> current_angles() const {
    return {.yaw = yaw_servo.angle.current, .pitch = pitch_servo.angle.current};
  }

  GimbalValues<float> target_angles() const {
    return {.yaw = yaw_servo.angle.target, .pitch = pitch_servo.angle.target};
  }

  Vector3 angular_velocity(Frame frame) const {
    Vector3 value = {.y = pitch_servo.speed.current, .z = yaw_servo.speed.current};
    if (frame == Frame::fixed)
      value = value + chassis_imu.angular_velocity;
    return value;
  }

  GimbalValues<float> current_speeds() const {
    return {.yaw = yaw_servo.speed.current, .pitch = pitch_servo.speed.current};
  }

  GimbalValues<float> target_speeds() const {
    return {.yaw = yaw_servo.speed.target, .pitch = pitch_servo.speed.target};
  }

  void enable(bool value) { yaw_servo.enabled.desired = pitch_servo.enabled.desired = value; }

  void follow_chassis(bool value) {
    spdlog::info("[Chassis] follow_chassis -> {}", value);
    if (value != following_chassis) {
      following_chassis = value;
      reset_target_attitude();
    }
  }

  void reset_target_attitude() {
    spdlog::info("[Chassis] reset_target_attitude [{}]", following_chassis);
    if (following_chassis) {
      target_attitude = attitude(Frame::chassis);
    } else {
      target_attitude = attitude(Frame::fixed);
    }
  }

  void set_mode(Mode _mode) {
    if (mode != _mode) {
      mode = _mode;
      if (mode == Mode::attitude_mode) {
        reset_target_attitude();
      }
    }
  }

  GimbalValues<float> fixed_angles(GimbalValues<float> value, GimbalValues<Frame> frame) const {
    GimbalValues<float> chassis_angles = chassis_attitude;
    GimbalValues<float> servo_angles = current_angles();
    for (size_t i = 0; i < value.size; i++) {
      if (frame[i] == Frame::chassis) {
        value[i] = value[i] + chassis_angles[i];
      } else if (frame.yaw == Frame::gimbal) {
        value[i] = value[i] + chassis_angles[i] + servo_angles[i];
      } else {
        float delta = normalize(value[i] - chassis_angles[i] - servo_angles[i]);
        const Servo *servo;
        if (i == 0) {
          servo = &yaw_servo;
        } else {
          servo = &pitch_servo;
        }
        if (servo_angles[i] + delta < servo->min_angle) {
          delta += 2 * M_PI;
        } else if (servo_angles[i] + delta > servo->max_angle) {
          delta -= 2 * M_PI;
        }
        value[i] = delta + chassis_angles[i] + servo_angles[i];
      }
    }
    return value;
  }
};

#endif  //  INCLUDE_ROBOT_GIMBAL_HPP_ */
