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
  chassis.wheel_speeds.current = chassis.wheel_speeds.target;
  chassis.wheel_angles = chassis.wheel_angles + chassis.wheel_speeds.current * last_time_step;

  std::map<unsigned, float> angles;
  for (auto const &[i, servo] : connected_servos) {
    angles[i] = servo->angle.current;
    servo->angle.current = std::clamp(servo->angle.current + servo->speed.target * time_step,
                                      servo->min_angle, servo->max_angle);
  }
  if (has_arm) {
    arm.limit_motor_angles(CURRENT);
  }
  for (auto const &[i, servo] : connected_servos) {
    servo->speed.current = (servo->angle.current - angles[i]) / time_step;
  }
  if (has_gripper) {
    if (gripper.state.current != gripper.state.target) {
      spdlog::info("Gripper -> {}", gripper.state.target);
    }
    gripper.state.current = gripper.state.target;
  }
  // This clamping is a mess
  // if (has_gimbal) {
  //   for (auto servo : {&gimbal.yaw_servo, &gimbal.pitch_servo}) {
  //     float angle = servo->angle.current;
  //     servo->angle.current = std::clamp(servo->angle.current + servo->speed.target *
  //     last_time_step,
  //                                       servo->min_angle, servo->max_angle);
  //     servo->speed.current = (servo->angle.current - angle) / last_time_step;
  //   }
  // }
  Robot::do_step(time_step);
}

WheelSpeeds DummyRobot::read_wheel_speeds() const { return chassis.wheel_speeds.current; }
void DummyRobot::forward_target_wheel_speeds(const WheelSpeeds &) {}
WheelValues<float> DummyRobot::read_wheel_angles() const { return chassis.wheel_angles; }

std::vector<uint8_t> DummyRobot::read_camera_image() const {
  static unsigned seq = 0;
  seq = (seq + 1) % camera.width;
  return generate_strip_image(seq, seq + 10, camera.width, camera.height);
}

void DummyRobot::forward_chassis_led(size_t index, const Color &color) {}
void DummyRobot::forward_gimbal_led(size_t index, size_t part, const Color &color) {}
void DummyRobot::forward_blaster_led(float value) const {}
IMU DummyRobot::read_imu() const {
  IMU imu;
  imu.acceleration = {0, 0, 9.81};
  // use odom as the source of velocity
  imu.angular_velocity = {0, 0, chassis.get_twist(Frame::body).theta};
  imu.attitude = {0, 0, 0};
  return imu;
}

bool DummyRobot::forward_camera_resolution(unsigned width, unsigned height) {
  camera.width = width;
  camera.height = height;
  return true;
}

#define TAU 0.25

void DummyRobot::forward_target_servo_angle(size_t index, float angle) {
  Servo *servo = connected_servos[index];
  float speed = (angle - servo->angle.current) / TAU;
  forward_target_servo_speed(index, speed);
  // Hack to force control update
  servo->angle.target = nanf("");
}

void DummyRobot::forward_target_servo_speed(size_t index, float speed) {
  Servo *servo = connected_servos[index];
  servo->speed.target = std::clamp(speed, -servo->max_speed, servo->max_speed);
}

float DummyRobot::read_servo_angle(size_t index) const {
  return connected_servos.at(index)->angle.current;
}

float DummyRobot::read_servo_speed(size_t index) const {
  return connected_servos.at(index)->speed.current;
}

void DummyRobot::forward_servo_mode(size_t index, Servo::Mode mode) {}

void DummyRobot::forward_servo_enabled(size_t index, bool value) {}

void DummyRobot::forward_target_gripper(Gripper::Status state, float power) {}

Gripper::Status DummyRobot::read_gripper_state() const { return gripper.state.current; }

DetectedObjects DummyRobot::read_detected_objects() const {
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

hit_event_t DummyRobot::read_hit_events() const { return {{.type = 0, .index = 0}}; }

ir_event_t DummyRobot::read_ir_events() const { return {{}}; }

float DummyRobot::read_tof(size_t index) const {
  return 0.89;
}
