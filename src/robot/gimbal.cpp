#include "robot/gimbal.hpp"

static const float tau = 0.5;

void Gimbal::update_control(float time_step, Attitude attitude, IMU imu) {
  chassis_attitude = attitude;
  chassis_imu = imu;
  // spdlog::info("[Gimbal] chassis {}, {}", attitude, imu);
  // spdlog::info("[Gimbal] servo {}, {}", yaw_servo.angle.current, pitch_servo.angle.current);
  if (mode == Gimbal::Mode::attitude_mode) {
    // spdlog::info("[Gimbal] target {} [action {}, follow {}]", target_attitude, performing_action,
    //              following_chassis);
    if (!following_chassis || performing_action) {
      float yaw_speed_world =
          (target_attitude.yaw - yaw_servo.angle.current - chassis_attitude.yaw) / tau;
      if (performing_action) {
        yaw_speed_world = std::clamp(yaw_speed_world, -control_speed.yaw, control_speed.yaw);
      }
      // spdlog::info("[Gimbal] yaw_speed_world {},  chassis speed {}", yaw_speed_world,
      //              chassis_imu.angular_velocity.z);
      yaw_servo.speed.desired =
          -chassis_imu.angular_velocity.z * (1 - time_step / tau) + yaw_speed_world;
      yaw_servo.mode.desired = Servo::SPEED;
      float pitch_speed_world =
          (target_attitude.pitch - pitch_servo.angle.current - chassis_attitude.pitch) / tau;
      if (performing_action) {
        pitch_speed_world =
            std::clamp(pitch_speed_world, -control_speed.pitch, control_speed.pitch);
      }
      pitch_servo.speed.desired =
          -chassis_imu.angular_velocity.y * (1 - time_step / tau) + pitch_speed_world;
      pitch_servo.mode.desired = Servo::SPEED;
      // spdlog::info("[Gimbal] desired yaw speed {},  desired pitch speed {}",
      //              yaw_servo.speed.desired, pitch_servo.speed.desired);
      float _speed;
      if (yaw_servo.will_pass_limits(time_step, _speed))
        target_attitude.yaw = yaw_servo.angle.current + chassis_attitude.yaw;
      if (pitch_servo.will_pass_limits(time_step, _speed))
        target_attitude.pitch = pitch_servo.angle.current + chassis_attitude.pitch;
    } else {
      yaw_servo.angle.desired = target_attitude.yaw;
      yaw_servo.mode.desired = Servo::ANGLE;
      pitch_servo.angle.desired = target_attitude.pitch;
      pitch_servo.mode.desired = Servo::ANGLE;
    }
  } else if (mode == Gimbal::Mode::velocity_mode) {
    // Velocity control (in world frame)
    Vector3 omega = target_angular_velocity_in_gimbal_fixed - chassis_imu.angular_velocity;

    // DONE(Jerome): Set to 0 or switch to attitude_mode if it is passing its limits

    yaw_servo.speed.desired = omega.z;
    yaw_servo.mode.desired = Servo::SPEED;
    pitch_servo.speed.desired = omega.y;
    pitch_servo.mode.desired = Servo::SPEED;
    float _speed;
    if (yaw_servo.will_pass_limits(time_step, _speed))
      target_angular_velocity_in_gimbal_fixed.z = 0;
    if (pitch_servo.will_pass_limits(time_step, _speed))
      target_angular_velocity_in_gimbal_fixed.y = 0;
  }
}
