#include <algorithm>

#include "spdlog/spdlog.h"

#include "coppeliasim_robot.hpp"

void CoppeliaSimRobot::update_target_wheel_speeds(WheelSpeeds &speeds) {
  for (size_t i = 0; i < 4; i++) {
    spdlog::debug("Set wheel joint {} speed to {}", i, speeds[i]);
    simSetJointTargetVelocity(wheel_joint_handles[i], speeds[i]);
  }
}

void CoppeliaSimRobot::update_led_colors(LEDColors &colors) {
  for (size_t i = 0; i < 4; i++) {
    simFloat color[3] = {colors[i].r, colors[i].g, colors[i].b};
    spdlog::debug("Set led {} color to {} {} {}", i, color[0], color[1], color[2]);
    simSetShapeColor(led_handles[i], nullptr, sim_colorcomponent_emission, color);
  }
}

WheelSpeeds CoppeliaSimRobot::read_wheel_speeds() {
  WheelSpeeds value;
  for (size_t i = 0; i < 4; i++) {
    simGetObjectFloatParameter(wheel_joint_handles[i], sim_jointfloatparam_velocity, &value[i]);
  }
  return value;
}

WheelValues<float> CoppeliaSimRobot::read_wheel_angles() {
  WheelValues<float> value;
  for (size_t i = 0; i < 4; i++) {
    simFloat rvalue;
    simGetJointPosition(wheel_joint_handles[i], &rvalue);
    value[i] = rvalue;
  }
  return value;
}

// DONE(jerome): add force sensor?, simulate gyro/magnetometer
// or maybe just get velocity and compute acceleration in the base class
IMU CoppeliaSimRobot::read_imu() { return imu; }

void CoppeliaSimRobot::has_read_accelerometer(float x, float y, float z) {
  imu.acceleration = {x, y, z};
}

void CoppeliaSimRobot::has_read_gyro(float x, float y, float z) {
  imu.angular_velocity = {x, y, z};
}

void CoppeliaSimRobot::update_orientation(float alpha, float beta, float gamma) {
  attitude.roll = alpha;
  attitude.pitch = beta;
}

std::vector<uint8_t> CoppeliaSimRobot::read_camera_image() {
  if (!camera_handle)
    return {};
  simHandleVisionSensor(camera_handle, nullptr, nullptr);
  simInt width = 0;
  simInt height = 0;
  simUChar *buffer = simGetVisionSensorCharImage(camera_handle, &width, &height);
  if (width != camera.width || height != camera.height) {
    spdlog::warn("Skip frame because of uncorrect size ({}, {}) vs desired size ({}, {})", width,
                 height, camera.width, camera.height);
    return {};
  }
  unsigned size = width * height * 3;
  // std::vector image(buffer, buffer + size);
  std::vector<uint8_t> image;
  image.reserve(size);
  simInt resolution[2] = {width, height};
  simTransformImage(buffer, resolution, 4, nullptr, nullptr, nullptr);

  std::copy(buffer, buffer + size, std::back_inserter(image));
  // image = std::vector(image);
  simReleaseBuffer((const simChar *)buffer);
  spdlog::debug("Got a {} x {} from CoppeliaSim", width, height);
  return image;
}

bool CoppeliaSimRobot::set_camera_resolution(unsigned width, unsigned height) {
  if (!camera_handle)
    return false;
  simInt image_size[2]{};
  simGetVisionSensorResolution(camera_handle, image_size);
  if (width != image_size[0] || height != image_size[1]) {
    simSetObjectInt32Parameter(camera_handle, sim_visionintparam_resolution_x, width);
    simSetObjectInt32Parameter(camera_handle, sim_visionintparam_resolution_y, height);
    spdlog::warn("Changing camera resolution from ({}, {}) to ({}, {})", image_size[0],
                 image_size[1], width, height);
    return true;
  }
  return true;
}

void CoppeliaSimRobot::update_target_servo_angles(const ServoValues<float> &angles) {
  if (servo_motor) {
    simSetJointTargetPosition(servo_motor.right, angles.right);
    simSetJointTargetPosition(servo_motor.left, angles.left);
  }
}

ServoValues<float> CoppeliaSimRobot::read_servo_angles() {
  if (servo_motor) {
    ServoValues<float> angles;
    simGetJointPosition(servo_motor.right, &angles.right);
    simGetJointPosition(servo_motor.left, &angles.left);
    return angles;
  }
  return {};
}

ServoValues<float> CoppeliaSimRobot::read_servo_speeds() {
  if (servo_motor) {
    ServoValues<float> speeds;
    simGetObjectFloatParameter(servo_motor.right, sim_jointfloatparam_velocity, &speeds.right);
    simGetObjectFloatParameter(servo_motor.left, sim_jointfloatparam_velocity, &speeds.left);
    return speeds;
  }
  return {};
}

void CoppeliaSimRobot::update_target_gripper(Robot::GripperStatus state, float power) {
  if (gripper_target_signal.empty()) {
    spdlog::warn("Gripper not available");
  }
  simSetIntegerSignal(gripper_target_signal.data(), static_cast<int>(state));
}

Robot::GripperStatus CoppeliaSimRobot::read_gripper_state() {
  if (gripper_state_signal.empty()) {
    spdlog::warn("Gripper not available");
    return Robot::GripperStatus::pause;
  }
  simInt value;
  simGetIntegerSignal(gripper_state_signal.data(), &value);
  return Robot::GripperStatus(value);
}

DetectedObjects CoppeliaSimRobot::read_detected_objects() { return {}; }

hit_event_t CoppeliaSimRobot::read_hit_events() { return {}; }
