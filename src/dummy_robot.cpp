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

void DummyRobot::do_step(float time_step) {
  wheel_angles = get_wheel_angles() + target_wheel_speed * last_time_step;
  ServoValues<float> angles;
  for (size_t i = 0; i < 2; i++) {
    angles[i] = servos[i].angle + servos[i].target_speed * time_step;
  }
  angles = limit_servo_angles(angles);
  for (size_t i = 0; i < 2; i++) {
    servos[i].speed = (angles[i] - servos[i].angle) / time_step;
    servos[i].angle = angles[i];
  }
  Robot::do_step(time_step);
}

std::vector<uint8_t> DummyRobot::read_camera_image() {
  static unsigned seq = 0;
  seq = (seq + 1) % camera.width;
  return generate_strip_image(seq, seq + 10, camera.width, camera.height);
}

void DummyRobot::update_target_wheel_speeds(WheelSpeeds &speeds) {}

WheelSpeeds DummyRobot::read_wheel_speeds() { return target_wheel_speed; }

WheelValues<float> DummyRobot::read_wheel_angles() { return wheel_angles; }

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

#define TAU 0.25

void DummyRobot::update_target_servo_angle(size_t index, float angle) {
  Servo *servo = &servos[index];
  float speed = (angle - servo->angle) / TAU;
  update_target_servo_speed(index, speed);
  // Hack to force control update
  servo->target_angle = nanf("");
}

void DummyRobot::update_target_servo_speed(size_t index, float speed) {
  Servo *servo = &servos[index];
  servo->target_speed = std::clamp(speed, -Servo::MAX_SPEED, Servo::MAX_SPEED);
}

float DummyRobot::read_servo_angle(size_t index) { return servos[index].angle; }

float DummyRobot::read_servo_speed(size_t index) { return servos[index].speed; }

void DummyRobot::update_servo_mode(size_t index, Servo::Mode mode) {}

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

ir_event_t DummyRobot::read_ir_events() { return {{}}; }

std::vector<ToFReading> DummyRobot::read_tof() { return {{.active = true, .distance = 0.89}}; }
