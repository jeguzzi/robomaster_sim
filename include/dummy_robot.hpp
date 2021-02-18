#ifndef INCLUDE_DUMMY_ROBOT_HPP_
#define INCLUDE_DUMMY_ROBOT_HPP_

#include <vector>

#include "robot.hpp"

class DummyRobot : public Robot {
 public:
  DummyRobot()
      : Robot() {}
  ~DummyRobot() {}
  void update_led_colors(LEDColors &);
  void update_target_wheel_speeds(WheelSpeeds &);
  WheelSpeeds read_wheel_speeds();
  WheelSpeeds read_wheel_angles();
  IMU read_imu();
  std::vector<uint8_t> read_camera_image();
  bool set_camera_resolution(unsigned width, unsigned height);
  void update_target_servo_angle(size_t index, float angle);
  void update_target_servo_speed(size_t index, float speed);
  float read_servo_angle(size_t index);
  float read_servo_speed(size_t index);
  void update_servo_mode(size_t index, Servo::Mode mode);
  void update_target_gripper(GripperStatus state, float power);
  GripperStatus read_gripper_state();
  DetectedObjects read_detected_objects();
  hit_event_t read_hit_events();
  ir_event_t read_ir_events();
  std::vector<ToFReading> read_tof();
  void do_step(float time_step);
};

#endif  // INCLUDE_DUMMY_ROBOT_HPP_
