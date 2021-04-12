#ifndef INCLUDE_ROBOT_SERVO_HPP_
#define INCLUDE_ROBOT_SERVO_HPP_

#include "../utils.hpp"

struct Servo {
  enum Mode { SPEED = 1, ANGLE = 0 };

  // TODO(Jerome): min_angle/max_angle/max_speed are placeholders
  Servo(bool enabled = true, float min_angle = deg2rad(-1800), float max_angle = deg2rad(1800),
        float max_speed = deg2rad(720))
      : min_angle(min_angle)
      , max_angle(max_angle)
      , max_speed(max_speed)
      , control_speed(max_speed)
      , angle(0.0)
      , speed(0.0)
      , mode(Mode::ANGLE)
      , enabled(enabled) {}

  float distance() const { return std::abs(normalize(angle.current - angle.desired)); }
  float min_angle;
  float max_angle;
  float max_speed;
  float control_speed;
  ControllableValue<float> angle;
  ControllableValue<float> speed;
  ControllableValue<Mode> mode;
  ControllableValue<bool> enabled;

  float desired_speed(float time_step) const {
    if (enabled.target && !will_pass_limits(time_step))
      return speed.desired;
    return 0.0f;
  }

  float get_desired_angle(float time_step) const {
    if (mode.current == SPEED) {
      return angle.current + desired_speed(time_step) * time_step;
    }
    return angle.desired;
  }

  void set_desired_angle(float value, float time_step) {
    if (mode.current == SPEED) {
      if (enabled.target)
        speed.desired = (value - angle.current) / time_step;
      return;
    }
    angle.desired = value;
  }

  bool will_pass_limits(float time_step) const {
    float nangle = speed.desired * time_step + angle.current;
    return nangle < min_angle || nangle > max_angle;
  }
};

#endif  //  INCLUDE_ROBOT_SERVO_HPP_ */