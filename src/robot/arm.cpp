#include "robot/robot.hpp"

Vector3 Arm::forward_arm_kinematics(const ArmServoValues<float> &_angles) {
  ArmServoValues<float> angles = ZERO - _angles;
  Vector3 p = {cos(angles.right) - cos(angles.left), 0, sin(angles.right) - sin(angles.left)};
  return p * L + OFFSET;
}

Matrix2 Arm::Jacobian(ArmServoValues<float> angles) {
  return {L * sin(ZERO.right - angles.right), -L * sin(ZERO.left - angles.left),
          -L * cos(ZERO.right - angles.right), L * cos(ZERO.left - angles.left)};
}

ArmServoValues<float> Arm::inverse_arm_kinematics(const Vector3 &position,
                                                  const Vector3 &arm_position,
                                                  ArmServoValues<float> arm_angles, float k) {
  Vector3 diff = position - arm_position;
  Vector2 delta = {diff.x, diff.z};
  Matrix2 m = Jacobian(arm_angles);
  // spdlog::info("Jacobian {}", m);
  m = m.inverse();
  // spdlog::info("Inverse {}", m);
  Vector2 delta_angle = m * delta;
  // spdlog::info("inverse_arm_kinematics {} -> {}", delta, delta_angle);
  ArmServoValues<float> delta_servo_angles{k * delta_angle.x, k * delta_angle.y};
  ArmServoValues<float> angles = arm_angles + delta_servo_angles;
  // auto v = forward_arm_kinematics(angles);
  // spdlog::info("forward_arm_kinematics {} ~-> {}", angles, v);
  // if((v - position).norm() > 0.001){
  //   return inverse_arm_kinematics(position, v, angles);
  // }
  // spdlog::info("Converged", angles, v);
  return angles;
}

ArmServoValues<float> Arm::safe_motor_angles(const ArmServoValues<float> &target_values) {
  // spdlog::info("set_target_servo_angles {}", values);
  // Limits:
  // -0.2740 < right < 1.3840
  // -0.7993 < left (< 1.7313)
  // -0.3473 < right - left < 1.2147
  ArmServoValues<float> values = target_values;
  values.right = std::clamp(values.right, -0.2740f, 1.3840f);
  values.left = std::clamp(values.left, -0.7993f, 1.7313f);
  // distance from the (right - left) upper line
  float dist = (values.right - values.left - 1.2147) / sqrt(2);
  if (dist > 0) {
    // compute nearest point on the line by moving down along (1, -1) by distance
    values.right -= dist;
    values.left += dist;
  }
  // distance from the (right - left) lower line
  dist = (values.right - values.left + 0.3473) / sqrt(2);
  if (dist < 0) {
    // compute nearest point on the line by moving down along (1, -1) by distance
    values.right -= dist;
    values.left += dist;
  }
  return values;
}

void Arm::update_position() { position = forward_arm_kinematics(get_motors_angle()); }

void Arm::update_control(const Vector3 &target_position) {
  // no planning, just a dummy (direct) controller based on inverse kinematic
  auto angles = inverse_arm_kinematics(target_position, position, get_motors_angle());
  motors.left.angle.set_target(angles.left);
  motors.right.angle.set_target(angles.right);
  limit_motor_angles(DESIRED);
}

void Arm::limit_motor_angles(ValueType level) {
  set_motors_angle(safe_motor_angles(get_motors_angle(level)), level);
}

void Arm::limit_motors(float time_step) {
  ArmServoValues<float> desired_angles = get_motors_desired_angle(time_step);
  desired_angles = safe_motor_angles(desired_angles);
  set_motors_desired_angle(desired_angles, time_step);
}
