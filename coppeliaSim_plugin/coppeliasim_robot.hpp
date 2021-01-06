#ifndef COPPELIASIM_ROBOT_HPP
#define COPPELIASIM_ROBOT_HPP

#include <simPlusPlus/Lib.h>
#include  "robot.hpp"

class CoppeliaSimRobot : public Robot
{

public:
  CoppeliaSimRobot(
      WheelValues<simInt> _wheel_joint_handles,
      LEDValues<simInt> _led_handles) {
    wheel_joint_handles = _wheel_joint_handles;
    led_handles = _led_handles;
  }
  void update_led_colors(LEDColors &);
  void update_target_wheel_speeds(WheelSpeeds &);
  WheelSpeeds read_wheel_speeds();
  WheelValues<float> read_wheel_angles();
  IMU read_imu();

  void has_read_accelerometer(float, float, float);
  void has_read_gyro(float, float, float);
  void update_orientation(float, float, float);
private:
  WheelValues<simInt> wheel_joint_handles;
  LEDValues<simInt> led_handles;
};

#endif /* end of include guard: COPPELIASIM_ROBOT_HPP */
