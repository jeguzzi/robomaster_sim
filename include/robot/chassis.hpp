#ifndef INCLUDE_ROBOT_CHASSIS_HPP_
#define INCLUDE_ROBOT_CHASSIS_HPP_

#include "../utils.hpp"

enum Frame { odom = 0, body = 1 };

using WheelSpeeds = WheelValues<float>;

struct Chassis {
  static constexpr float axis_x = 0.1;
  static constexpr float axis_y = 0.1;
  static constexpr float wheel_radius = 0.05;

  Chassis()
      : wheel_angles()
      , wheel_speeds({})
      , imu()
      , odometry()
      , body_twist()
      , attitude()
      , wheel_motors_engaged(true) {}

  WheelValues<float> wheel_angles;
  // Control the angular speed of the wheels in [rad/s] with respect to robot y-axis
  ControllableValue<WheelSpeeds> wheel_speeds;
  IMU imu;
  Odometry odometry;
  Twist2D body_twist;
  Attitude attitude;
  bool wheel_motors_engaged;

  Twist2D get_twist(Frame frame) const;
  void set_target_velocity(const Twist2D &twist);
  void stop();
  void update_odometry(float time_step);
  void update_attitude(float time_step);

  WheelSpeeds get_wheel_speeds() const { return wheel_speeds.current; }
  WheelValues<float> get_wheel_angles() const { return wheel_angles; }
  Pose2D get_pose() const { return odometry.pose; }
  Attitude get_attitude() const { return attitude; }
  IMU get_imu() const { return imu; }
};

#endif  //  INCLUDE_ROBOT_CHASSIS_HPP_ */
