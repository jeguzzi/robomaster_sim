#include "spdlog/spdlog.h"

#include "dummy_robot.hpp"

static std::vector<uint8_t> generate_strip_image(unsigned i0, unsigned i1, unsigned width,
                                                 unsigned height) {
  unsigned size = width * height * 3;
  std::vector<uint8_t> buffer(size, 0);
  if (i0 > i1)
    i1 += width;
  for (size_t i = i0; i < i1; i++)
    for (size_t j = 0; j < height; j++)
      buffer[(3 * (j * width + i)) % size] = 255;
  return buffer;
}

std::vector<uint8_t> DummyRobot::read_camera_image() {
  static unsigned seq = 0;
  seq = (seq + 1) % camera.width;
  return generate_strip_image(seq, seq + 10, camera.width, camera.height);
}

void DummyRobot::update_target_wheel_speeds(WheelSpeeds &speeds) {}

WheelSpeeds DummyRobot::read_wheel_speeds() { return target_wheel_speed; }

WheelValues<float> DummyRobot::read_wheel_angles() {
  if (last_time_step) {
    return get_wheel_angles() + target_wheel_speed * last_time_step;
  }
  return {};
}

void DummyRobot::update_led_colors(LEDColors &colors) {}

IMU DummyRobot::read_imu() {
  IMU imu;
  imu.acceleration = {0, 0, 9.81};
  // use odom as the source of velocity
  imu.angular_velocity = {0, 0, get_twist(Robot::Frame::body).theta};
  return imu;
}

bool DummyRobot::set_camera_resolution(unsigned width, unsigned height) {
  camera.width = width;
  camera.height = height;
  return true;
}

void DummyRobot::update_target_servo_angles(const ServoValues<float> &angles) {}

ServoValues<float> DummyRobot::read_servo_angles() {
  return {0.5f * (get_servo_angles().right + target_servo_angles.right),
          0.5f * (get_servo_angles().left + target_servo_angles.left)};
}

ServoValues<float> DummyRobot::read_servo_speeds() {
  return {0.5f * (get_servo_angles().right + target_servo_angles.right) / last_time_step,
          0.5f * (get_servo_angles().left + target_servo_angles.left) / last_time_step};
}

void DummyRobot::update_target_gripper(Robot::GripperStatus state, float power) {}

Robot::GripperStatus DummyRobot::read_gripper_state() { return target_gripper_state; }

DetectedObjects DummyRobot::read_detected_objects() {
  // Fill with one object per type
  DetectedObjects objects;
  if (vision.is_enabled<DetectedObjects::Person>()) {
    objects.people = {DetectedObjects::Person({0.5, 0.5, 0.25, 0.25})};
  }
  if (vision.is_enabled<DetectedObjects::Gesture>()) {
    objects.gestures = {DetectedObjects::Gesture({0.5, 0.5, 0.25, 0.25}, 1)};
  }
  if (vision.is_enabled<DetectedObjects::Line>()) {
    objects.lines = {DetectedObjects::Line(0.5, 0.5, 0, 0)};
  }
  if (vision.is_enabled<DetectedObjects::Marker>()) {
    objects.markers = {DetectedObjects::Marker({0.5, 0.5, 0.25, 0.25}, 1, 1.0)};
  }
  if (vision.is_enabled<DetectedObjects::Robot>()) {
    objects.robots = {DetectedObjects::Robot({0.5, 0.5, 0.25, 0.25})};
  }
  return objects;
}

hit_event_t DummyRobot::read_hit_events() { return {{.index = 0, .type = 0}}; }

std::vector<ToFReading> DummyRobot::read_tof() { return {{.active = true, .distance = 0.89}}; }
