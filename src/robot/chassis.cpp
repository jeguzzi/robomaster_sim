#include "robot/robot.hpp"

static WheelSpeeds wheel_speeds_from_twist(const Twist2D &twist, float l = 0.2,
                                           float radius = 0.05) {
  WheelSpeeds value;
  value.front_left = (twist.x - twist.y - l * twist.theta) / radius;
  value.front_right = (twist.x + twist.y + l * twist.theta) / radius;
  value.rear_left = (twist.x + twist.y - l * twist.theta) / radius;
  value.rear_right = (twist.x - twist.y + l * twist.theta) / radius;
  return value;
}

static Twist2D twist_from_wheel_speeds(const WheelSpeeds &speeds, float l = 0.2,
                                       float radius = 0.05) {
  Twist2D value;
  value.x = 0.25 * (speeds.front_left + speeds.front_right + speeds.rear_left + speeds.rear_right) *
            radius;
  value.y = 0.25 *
            (-speeds.front_left + speeds.front_right + speeds.rear_left - speeds.rear_right) *
            radius;
  value.theta = 0.25 *
                (-speeds.front_left + speeds.front_right - speeds.rear_left + speeds.rear_right) *
                radius / l;
  return value;
}

Twist2D Chassis::get_twist(Frame frame) const {
  if (frame == Frame::odom)
    return odometry.twist;
  else
    return body_twist;
}

void Chassis::set_target_velocity(const Twist2D &twist) {
  WheelSpeeds speeds = wheel_speeds_from_twist(twist, axis_x + axis_y, wheel_radius);
  wheel_speeds.set_target(speeds);
}

void Chassis::stop() {
  WheelSpeeds speeds {0.0f};
  wheel_speeds.set_target(speeds);
}

void Chassis::update_odometry(float time_step) {
  // spdlog::info("[Chassis] Wheel speed {}", wheel_speeds.current);
  body_twist = twist_from_wheel_speeds(wheel_speeds.current);
  // spdlog::info("[Chassis] body_twist {}", body_twist);
  odometry.twist = body_twist.rotate_around_z(odometry.pose.theta);
  odometry.pose = odometry.pose + odometry.twist * time_step;
  // spdlog::info("update_odometry {}, {}, {}", body_twist, odometry.twist, odometry.pose);
}

void Chassis::update_attitude(float time_step) {
  // spdlog::info("[Chassis] imu {}", imu);
  attitude.yaw += time_step * imu.angular_velocity.z;
  attitude.pitch = imu.attitude.pitch;
  attitude.roll = imu.attitude.roll;
  // use [imu] attitude as a source for odometry angular data:
  odometry.twist.theta = imu.angular_velocity.z;
  odometry.pose.theta = attitude.yaw;
  // TODO(Jerome): body twist too
  // spdlog::info("update_attitude {}, {}", imu.attitude.yaw,  odometry.pose);
}
