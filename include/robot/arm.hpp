#ifndef INCLUDE_ROBOT_ARM_HPP_
#define INCLUDE_ROBOT_ARM_HPP_

#include "../utils.hpp"

#include "servo.hpp"

// CHANGED(Jerome): invert left motor direction ... here or in coppelia => in utils.hpp
// CHANGED(Jerome): modify zeros to match coppelia/model

struct Arm {
  // Length of an arm
  static constexpr float L = 0.12f;
  // The coordinate system of the arm is located at the middle top of the arm [trapeziodal] platfom
  // The end effector is longitudinally at start of edge of the gripper, at an altitude
  // about 1 cm below the arm end effector

  static constexpr ArmServoValues<float> ZERO{1.5359f, 2.6442f};
  // static constexpr ArmServoValues<float> ZERO{1.5508f, 2.6578f};
  //
  // pitch arm top (left motor): 27.72 deg != 28.5 deg in coppelia
  // pitch arm bottom (right motor): -88.85 deg != -88 deg in coppelia

  // TODO(Jerome): should be (0.074, 0.057)
  // Defined so to that (0.071, 0.057) is the point with maximal flection (like in the real RM)
  static constexpr Vector3 OFFSET{-0.00157f, 0.0f, 0.00523f};
  // static constexpr Vector3 OFFSET{-0.0007f, 0.0f, 0.0043f};
  // {0.0032f, 0.0f, 0.0179f};

  ArmServoValues<Servo &> motors;
  Vector3 position;

  static Vector3 forward_arm_kinematics(const ArmServoValues<float> &_angles);
  static Matrix2 Jacobian(ArmServoValues<float> angles);

  static ArmServoValues<float> inverse_arm_kinematics(const Vector3 &position,
                                                      const Vector3 &arm_position,
                                                      ArmServoValues<float> arm_angles,
                                                      float k = 0.2);

  static ArmServoValues<float> safe_motor_angles(const ArmServoValues<float> &target_values);

  Arm(Servo &left_motor, Servo &right_motor)
      : motors({.right = right_motor, .left = left_motor}) {
    // motors.right.min_angle = -0.2740f;
    // motors.right.max_angle = 1.3840f;
    // left motor limits depends on the right motor angle and are updated in `Arm::limit_motors`
  }

  ArmServoValues<float> get_motors_angle(ValueType type = CURRENT) const {
    return {.right = motors.right.angle.get(type), .left = motors.left.angle.get(type)};
  }

  void set_motors_angle(ArmServoValues<float> value, ValueType type = CURRENT) const {
    motors.left.angle.set(value.left, type);
    motors.right.angle.set(value.right, type);
  }

  void update_position();
  void update_control(const Vector3 &target_position);
  Vector3 get_position() const { return position; }
  float distance() const { return motors.right.distance() + motors.left.distance(); }
  void limit_motor_angles(ValueType);
  void limit_motors(float time_step);
  ArmServoValues<float> get_motors_desired_angle(float time_step) const {
    return {.right = motors.right.get_desired_angle(time_step),
            .left = motors.left.get_desired_angle(time_step)};
  }
  void set_motors_desired_angle(ArmServoValues<float> values, float time_step) {
    motors.left.set_desired_angle(values.left, time_step);
    motors.right.set_desired_angle(values.right, time_step);
  }
};

#endif  //  INCLUDE_ROBOT_ARM_HPP_ */
