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
  void update_target_servo_angles(const ServoValues<float> &angles);
  ServoValues<float> read_servo_angles();
  ServoValues<float> read_servo_speeds();
  void update_target_gripper(GripperStatus state, float power);
  GripperStatus read_gripper_state();
  DetectedObjects read_detected_objects();
  hit_event_t read_hit_events();
  std::vector<ToFReading> read_tof();
};

#endif  // INCLUDE_DUMMY_ROBOT_HPP_
