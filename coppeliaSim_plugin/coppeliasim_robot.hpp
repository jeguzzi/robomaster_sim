#ifndef COPPELIASIM_ROBOT_HPP
#define COPPELIASIM_ROBOT_HPP

#include <simPlusPlus/Lib.h>
#include  "robot.hpp"

class CoppeliaSimRobot : public Robot
{

public:
  CoppeliaSimRobot(
      WheelValues<simInt> _wheel_joint_handles,
      LEDValues<simInt> _led_handles,
      simInt _camera_handle,
      ServoValues<simInt> _servo_motor
  ) {
    wheel_joint_handles = _wheel_joint_handles;
    led_handles = _led_handles;
    camera_handle = _camera_handle;
    servo_motor = _servo_motor;
  }
  void update_led_colors(LEDColors &);
  void update_target_wheel_speeds(WheelSpeeds &);
  WheelSpeeds read_wheel_speeds();
  WheelValues<float> read_wheel_angles();
  IMU read_imu();
  std::vector<unsigned char> read_camera_image();
  bool set_camera_resolution(unsigned width, unsigned height);
  void has_read_accelerometer(float, float, float);
  void has_read_gyro(float, float, float);
  void update_orientation(float, float, float);
  void update_target_servo_angles(ServoValues<float> &angles);
  ServoValues<float> read_servo_angles();
private:
  WheelValues<simInt> wheel_joint_handles;
  LEDValues<simInt> led_handles;
  simInt camera_handle;
  ServoValues<simInt> servo_motor;
};

#endif /* end of include guard: COPPELIASIM_ROBOT_HPP */
