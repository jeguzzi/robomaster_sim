#include <algorithm>

#include "spdlog/spdlog.h"

#include "coppeliasim_robot.hpp"

// bool read_vector_from_signal(const std::string name, Vector3 *out) {
//   simInt buffer_lenght;
//   simChar *buffer = simGetStringSignal(name.data(), &buffer_lenght);
//   if (buffer && buffer_lenght == sizeof(Vector3)) {
//     simInt stack = simCreateStack();
//     simInt r = simUnpackTable(stack, buffer, buffer_lenght);
//     if (r < 0) {
//       return false;
//     }
//     r = simGetStackFloatTable(stack, &(out->x), 3);
//     return (r >= 0);
//   }
// }

bool read_vector_from_signal(const std::string name, Vector3 *out) {
  simInt buffer_lenght;
  simChar *buffer = simGetStringSignal(name.data(), &buffer_lenght);
  if (buffer && buffer_lenght == sizeof(Vector3)) {
    memcpy(&(out->x), buffer, buffer_lenght);
    return true;
  }
  return false;
}

void CoppeliaSimRobot::forward_target_wheel_speeds(const WheelSpeeds &speeds) {
  for (size_t i = 0; i < speeds.size; i++) {
    spdlog::debug("Set wheel joint {} speed to {}", i, speeds[i]);
    simSetJointTargetVelocity(wheel_joint_handles[i], speeds[i]);
  }
}

void CoppeliaSimRobot::forward_chassis_led(size_t index, const Color &rgb) {
  if (index >= chassis_led_handles.size) {
    spdlog::warn("Unknown chassis led {}", index);
    return;
  }
  simFloat color[3] = {rgb.r, rgb.g, rgb.b};
  spdlog::debug("Set led {} color to {} {} {}", index, color[0], color[1], color[2]);
  simSetShapeColor(chassis_led_handles[index], nullptr, sim_colorcomponent_emission, color);
}

void CoppeliaSimRobot::forward_gimbal_led(size_t index, size_t part, const Color &rgb) {
  if (index >= gimbal_led_handles.size || gimbal_led_handles[index].size() <= part) {
    spdlog::warn("Unknown gimbal led {}[{}]", index, part);
    return;
  }
  simFloat color[3] = {rgb.r, rgb.g, rgb.b};
  spdlog::debug("Set gimbal led {}-{} color to {} {} {} [{}]", index, part, color[0], color[1],
                color[2], gimbal_led_handles[index][part]);
  simSetShapeColor(gimbal_led_handles[index][part], nullptr, sim_colorcomponent_emission, color);
}

void CoppeliaSimRobot::forward_blaster_led(float value) const {
  if (blaster_light_handle <= 0)
    return;
  const simFloat rgb[3] = {0.0f, std::clamp(value, 0.0f, 1.0f), 0.0f};
  if (value)
    simSetLightParameters(blaster_light_handle, 1, nullptr, rgb, nullptr);
  else
    simSetLightParameters(blaster_light_handle, 0, nullptr, nullptr, nullptr);
}

WheelSpeeds CoppeliaSimRobot::read_wheel_speeds() const {
  WheelSpeeds value;
  for (size_t i = 0; i < value.size; i++) {
    simGetObjectFloatParameter(wheel_joint_handles[i], sim_jointfloatparam_velocity, &value[i]);
  }
  return value;
}

WheelValues<float> CoppeliaSimRobot::read_wheel_angles() const {
  WheelValues<float> value;
  for (size_t i = 0; i < value.size; i++) {
    simFloat rvalue;
    simGetJointPosition(wheel_joint_handles[i], &rvalue);
    value[i] = rvalue;
  }
  return value;
}

// DONE(jerome): add force sensor?, simulate gyro/magnetometer
// or maybe just get velocity and compute acceleration in the base class
IMU CoppeliaSimRobot::read_imu() const {
  IMU imu;
  if (!read_vector_from_signal(gyro_signal, &imu.angular_velocity)) {
    spdlog::warn("Could not read signal {}", gyro_signal);
    return {};
  }
  if (!read_vector_from_signal(accelerometer_signal, &imu.acceleration)) {
    spdlog::warn("Could not read signal {}", accelerometer_signal);
    return {};
  }
  // TODO(IMU has no attitude->should move it in a separate method)
  simFloat euler_angles[3];
  if (simGetObjectOrientation(imu_handle, -1, euler_angles) < 0) {
    spdlog::warn("Could not get orientation of object {} [imu_handle]", imu_handle);
    return {};
  }
  imu.attitude.roll = euler_angles[0];
  imu.attitude.pitch = euler_angles[1];
  return imu;
}

// void CoppeliaSimRobot::has_read_accelerometer(float x, float y, float z) {
//   chassis.imu.acceleration = {x, y, z};
// }

// void CoppeliaSimRobot::has_read_gyro(float x, float y, float z) {
//   // spdlog::info("[CS] has_read_gyro {} {} {}", x, y, z);
//   chassis.imu.angular_velocity = {x, y, z};
// }

// void CoppeliaSimRobot::update_orientation(float alpha, float beta, float gamma) {
//   chassis.attitude.roll = alpha;
//   chassis.attitude.pitch = beta;
// }

std::vector<uint8_t> CoppeliaSimRobot::read_camera_image() const {
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

bool CoppeliaSimRobot::forward_camera_resolution(unsigned width, unsigned height) {
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

void CoppeliaSimRobot::forward_target_servo_angle(size_t index, float angle) {
  if (servo_handles.at(index) > 0) {
    simSetJointTargetPosition(servo_handles.at(index), angle);
  }
}

void CoppeliaSimRobot::forward_servo_mode(size_t index, Servo::Mode mode) {
  if (servo_handles.at(index) > 0) {
    if (mode == Servo::ANGLE) {
      simSetObjectInt32Parameter(servo_handles.at(index), sim_jointintparam_ctrl_enabled, 1);
    } else {
      simSetObjectInt32Parameter(servo_handles.at(index), sim_jointintparam_ctrl_enabled, 0);
    }
  }
}

void CoppeliaSimRobot::forward_servo_enabled(size_t index, bool value) {
  if (servo_handles.at(index) > 0) {
    simSetObjectInt32Parameter(servo_handles.at(index), sim_jointintparam_motor_enabled, value);
  }
}

void CoppeliaSimRobot::forward_target_servo_speed(size_t index, float speed) {
  if (servo_handles.at(index) > 0) {
    // spdlog::info("update_target_servo_speed {} {}", index, speed);
    simSetJointTargetVelocity(servo_handles.at(index), speed);
  }
}

float CoppeliaSimRobot::read_servo_angle(size_t index) const {
  float angle = 0;
  if (servo_handles.at(index) > 0) {
    simGetJointPosition(servo_handles.at(index), &angle);
  }
  return angle;
}

float CoppeliaSimRobot::read_servo_speed(size_t index) const {
  float speed = 0;
  if (servo_handles.at(index) > 0) {
    simGetObjectFloatParameter(servo_handles.at(index), sim_jointfloatparam_velocity, &speed);
  }
  return speed;
}

void CoppeliaSimRobot::forward_target_gripper(Gripper::Status state, float power) {
  if (gripper_target_signal.empty()) {
    spdlog::warn("Gripper not available");
  }
  simSetIntegerSignal(gripper_target_signal.data(), static_cast<int>(state));
}

Gripper::Status CoppeliaSimRobot::read_gripper_state() const {
  if (gripper_state_signal.empty()) {
    spdlog::warn("Gripper not available");
    return Gripper::Status::pause;
  }
  simInt value;
  simGetIntegerSignal(gripper_state_signal.data(), &value);
  return Gripper::Status(value);
}

DetectedObjects CoppeliaSimRobot::read_detected_objects() const { return {}; }

hit_event_t CoppeliaSimRobot::read_hit_events() const { return {}; }

float CoppeliaSimRobot::read_tof(size_t index) const {
  if (!tof_handles.count(index)) return 0.0f;
  simInt h = tof_handles.at(index);
  simFloat p[4];
  simInt r = simCheckProximitySensor(h, sim_handle_all, p);
  if (r == 1) {
    return p[3];
  } else if (r == 0) {
    return -1.0;
  }
  return 0.0f;
}

ir_event_t CoppeliaSimRobot::read_ir_events() const { return {}; }

void CoppeliaSimRobot::enable_tof(size_t index, simInt sensor_handle) {
  if (index < MAX_NUMBER_OF_TOF_SENSORS) {
    tof_handles[index] = sensor_handle;
    tof.set_enable(index, true);
    has_tof = true;
    spdlog::info("Enabled distance sensor");
  }
}
