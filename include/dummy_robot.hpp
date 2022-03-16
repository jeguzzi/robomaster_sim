#ifndef INCLUDE_DUMMY_ROBOT_HPP_
#define INCLUDE_DUMMY_ROBOT_HPP_

#include <vector>

#include "robot/robot.hpp"

class DummyRobot : public Robot {
 public:
  explicit DummyRobot(bool _has_arm = false, bool _has_gripper = false,
                      ServoValues<bool> _has_servo = {}, bool _has_gimbal = false,
                      bool _has_camera = false, bool _has_tof = false)
      : Robot(_has_arm, _has_gripper, _has_servo, _has_gimbal, _has_camera, _has_tof) {}
  ~DummyRobot() {}
  WheelSpeeds read_wheel_speeds() const;
  void forward_target_wheel_speeds(const WheelSpeeds &);
  WheelValues<float> read_wheel_angles() const;
  void forward_target_wheel_angles(const WheelValues<float> &);
  IMU read_imu() const;
  void forward_chassis_led(size_t index, const Color &);
  void forward_gimbal_led(size_t index, size_t part, const Color &);
  Gripper::Status read_gripper_state() const;
  void forward_target_gripper(Gripper::Status state, float power);
  bool forward_camera_resolution(unsigned width, unsigned height);
  Image read_camera_image() const;
  hit_event_t read_hit_events() const;
  ir_event_t read_ir_events() const;
  DetectedObjects read_detected_objects() const;
  void forward_target_servo_angle(size_t index, float angle);
  void forward_target_servo_speed(size_t index, float speed);
  float read_servo_angle(size_t index) const;
  float read_servo_speed(size_t index) const;
  void forward_servo_mode(size_t index, Servo::Mode mode);
  void forward_servo_enabled(size_t index, bool value);
  float read_tof(size_t index) const;
  void forward_target_gimbal_speed(const GimbalValues<float> &speed);
  void forward_target_gimbal_angle(const GimbalValues<float> &angle);
  void forward_blaster_led(float value) const;
  void do_step(float time_step);
};

#endif  // INCLUDE_DUMMY_ROBOT_HPP_
