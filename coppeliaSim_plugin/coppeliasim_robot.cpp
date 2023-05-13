#include <algorithm>

#include "spdlog/spdlog.h"

#include "coppeliasim_robot.hpp"

// bool read_vector_from_signal(const std::string name, Vector3 *out) {
//   int buffer_lenght;
//   char *buffer = simGetStringSignal(name.data(), &buffer_lenght);
//   if (buffer && buffer_lenght == sizeof(Vector3)) {
//     int stack = simCreateStack();
//     int r = simUnpackTable(stack, buffer, buffer_lenght);
//     if (r < 0) {
//       return false;
//     }
//     r = simGetStackFloatTable(stack, &(out->x), 3);
//     return (r >= 0);
//   }
// }

bool read_vector_from_signal(const std::string name, Vector3 *out) {
  int buffer_lenght;
  char *buffer = simGetStringSignal(name.data(), &buffer_lenght);
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
  float color[3] = {rgb.r, rgb.g, rgb.b};
  spdlog::debug("Set led {} color to {} {} {}", index, color[0], color[1], color[2]);
  simSetShapeColor(chassis_led_handles[index], nullptr, sim_colorcomponent_emission, color);
}

void CoppeliaSimRobot::forward_gimbal_led(size_t index, size_t part, const Color &rgb) {
  if (index >= gimbal_led_handles.size || gimbal_led_handles[index].size() <= part) {
    spdlog::warn("Unknown gimbal led {}[{}]", index, part);
    return;
  }
  float color[3] = {rgb.r, rgb.g, rgb.b};
  spdlog::debug("Set gimbal led {}-{} color to {} {} {} [{}]", index, part, color[0], color[1],
                color[2], gimbal_led_handles[index][part]);
  simSetShapeColor(gimbal_led_handles[index][part], nullptr, sim_colorcomponent_emission, color);
}

void CoppeliaSimRobot::forward_blaster_led(float value) const {
  if (blaster_light_handle <= 0)
    return;
  const float rgb[3] = {0.0f, std::clamp(value, 0.0f, 1.0f), 0.0f};
  if (value)
    simSetLightParameters(blaster_light_handle, 1, nullptr, rgb, nullptr);
  else
    simSetLightParameters(blaster_light_handle, 0, nullptr, nullptr, nullptr);
}

WheelSpeeds CoppeliaSimRobot::read_wheel_speeds() const {
  WheelSpeeds value;
  for (size_t i = 0; i < value.size; i++) {
    // I need to read a simFloat because from version 4.5, coppeliaSim uses double instead of floats
    // in most apis.
    simFloat v;
    simGetObjectFloatParam(wheel_joint_handles[i], sim_jointfloatparam_velocity, &v);
    value[i] = v;
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
  int resolution[2]{};
#if SIM_PROGRAM_VERSION_NB >= 40400
  unsigned char *buffer = simGetVisionSensorImg(camera_handle, 0, 0.0, nullptr, nullptr, resolution);
#else
  unsigned char *buffer = simGetVisionSensorCharImage(handle, &resolution[0], &resolution[1]);
#endif
  const int width = resolution[0];
  const int height = resolution[1];
  if (width != camera.width || height != camera.height) {
    spdlog::warn("Skip frame because of uncorrect size ({}, {}) vs desired size ({}, {})", width,
                 height, camera.width, camera.height);
    return {};
  }
  unsigned size = width * height * 3;
  // std::vector image(buffer, buffer + size);
  std::vector<uint8_t> image;
  image.reserve(size);
  // int resolution[2] = {width, height};
  simTransformImage(buffer, resolution, 4, nullptr, nullptr, nullptr);

  std::copy(buffer, buffer + size, std::back_inserter(image));
  // image = std::vector(image);
  simReleaseBuffer((const char *)buffer);
  spdlog::debug("Got a {} x {} from CoppeliaSim", width, height);
  return image;
}

bool CoppeliaSimRobot::forward_camera_resolution(unsigned width, unsigned height) {
  if (!camera_handle)
    return false;
  int image_size[2]{};
#if SIM_PROGRAM_VERSION_NB >= 40500
  simGetVisionSensorRes(camera_handle, image_size);
#else
  simGetVisionSensorResolution(camera_handle, image_size);
#endif
  if (width != image_size[0] || height != image_size[1]) {
    simSetObjectInt32Param(camera_handle, sim_visionintparam_resolution_x, width);
    simSetObjectInt32Param(camera_handle, sim_visionintparam_resolution_y, height);
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
      simSetObjectInt32Param(servo_handles.at(index), sim_jointintparam_ctrl_enabled, 1);
    } else {
      simSetObjectInt32Param(servo_handles.at(index), sim_jointintparam_ctrl_enabled, 0);
    }
  }
}

void CoppeliaSimRobot::forward_servo_enabled(size_t index, bool value) {
  if (servo_handles.at(index) > 0) {
    simSetObjectInt32Param(servo_handles.at(index), sim_jointintparam_motor_enabled, value);
  }
}

void CoppeliaSimRobot::forward_target_servo_speed(size_t index, float speed) {
  if (servo_handles.at(index) > 0) {
    // spdlog::info("update_target_servo_speed {} {}", index, speed);
    simSetJointTargetVelocity(servo_handles.at(index), speed);
  }
}

float CoppeliaSimRobot::read_servo_angle(size_t index) const {
  simFloat angle = 0;
  if (servo_handles.at(index) > 0) {
    simGetJointPosition(servo_handles.at(index), &angle);
  }
  return angle;
}

float CoppeliaSimRobot::read_servo_speed(size_t index) const {
  simFloat speed = 0;
  if (servo_handles.at(index) > 0) {
    simGetObjectFloatParam(servo_handles.at(index), sim_jointfloatparam_velocity, &speed);
  }
  return speed;
}

void CoppeliaSimRobot::forward_target_gripper(Gripper::Status state, float power) {
  if (gripper_target_signal.empty()) {
    spdlog::warn("Gripper not available");
  }
#if SIM_PROGRAM_VERSION_NB >= 40300
  simSetInt32Signal(gripper_target_signal.data(), static_cast<int>(state));
#else
  simSetIntegerSignal(gripper_target_signal.data(), static_cast<int>(state));
#endif
}

Gripper::Status CoppeliaSimRobot::read_gripper_state() const {
  if (gripper_state_signal.empty()) {
    spdlog::warn("Gripper not available");
    return Gripper::Status::pause;
  }
  int value;
#if SIM_PROGRAM_VERSION_NB >= 40300
  simGetInt32Signal(gripper_target_signal.data(), &value);
#else  
  simGetIntegerSignal(gripper_state_signal.data(), &value);
#endif
  return Gripper::Status(value);
}


// static cv::Rect find_bounding_box(const cv::Mat & image, const cv::Scalar & color) {
//   // cv::Mat mask(image.size(), CV_8UC1);
//   cv::Mat mask;
//   cv::inRange(image, color, color, mask);
//   if (cv::countNonZero(mask)) {
//     cv::Mat locations;
//     cv::findNonZero(mask, locations);
//     return cv::boundingRect(locations);
//   }
//   return cv::Rect();
// }


struct ImageRect {
  uint16_t min_i, max_i, min_j, max_j;

  ImageRect(uint16_t i, uint16_t j) : min_i(i), max_i(i), min_j(j), max_j(j) {}

  void update(uint16_t i, uint16_t j) {
    min_i = std::min(min_i, i);
    min_j = std::min(min_j, j);
    max_i = std::max(max_i, i);
    max_j = std::max(max_j, j);
  }

  BoundingBox box(uint16_t w, uint16_t h) const {
    return BoundingBox(
        (float) (max_j + min_j) / w * 0.5, 1.0 - (float) (max_i + min_i) / h * 0.5,
        (float) (max_j - min_j + 1) / w, (float) (max_i - min_i + 1) / h);
  }
};

static DetectedObjects run_detector(const unsigned char * buffer, int width, int height,
                                    uint8_t mask) {
  std::map<uint8_t, ImageRect> boxes;
  buffer++;
  for (size_t i = 0; i < height; i++) {
    for (size_t j = 0; j < width; j++) {
      if (*buffer != 0) {
        auto box = boxes.find(*buffer);
        if (box == boxes.end()) {
          boxes.emplace(std::piecewise_construct, std::make_tuple(*buffer), std::make_tuple(i, j));
        } else {
          box->second.update(i, j);
        }
      }
      buffer+=3;
    }
  }
  DetectedObjects objects;
  for (auto & [k, v] : boxes) {
    if (k == DetectedObjects::Type::PERSON && (1 << k) & mask) {
      objects.people.emplace_back(v.box(width, height));
    }
    if (k == DetectedObjects::Type::ROBOT && (1 << k) & mask) {
      objects.robots.emplace_back(v.box(width, height));
    }
  }
  return objects;
}

DetectedObjects CoppeliaSimRobot::read_detected_objects() const {
  if (vision_handle <= 0) {
    spdlog::warn("No vision sensor");
    return {};
  }
  simHandleVisionSensor(vision_handle, nullptr, nullptr);
  int resolution[2]{};
#if SIM_PROGRAM_VERSION_NB >= 40400
  unsigned char *buffer = simGetVisionSensorImg(vision_handle, 0, 0.0, nullptr, nullptr, resolution);
#else
  unsigned char *buffer = simGetVisionSensorCharImage(vision_handle, &resolution[0], &resolution[1]);
#endif
  const int width = resolution[0];
  const int height = resolution[1];
  auto rs = run_detector(buffer, width, height, vision.get_enable());
  simReleaseBuffer((const char *)buffer);
  return rs;
}

hit_event_t CoppeliaSimRobot::read_hit_events() const { return {}; }

float CoppeliaSimRobot::read_tof(size_t index) const {
  if (!tof_handles.count(index)) return 0.0f;
  int h = tof_handles.at(index);
  simFloat p[4];
  int r = simCheckProximitySensor(h, sim_handle_all, p);
  if (r == 1) {
    return p[3];
  } else if (r == 0) {
    return -1.0;
  }
  return 0.0f;
}

ir_event_t CoppeliaSimRobot::read_ir_events() const { return {}; }

void CoppeliaSimRobot::enable_tof(size_t index, int sensor_handle) {
  if (index < MAX_NUMBER_OF_TOF_SENSORS && Robot::enable_tof(index)) {
    tof_handles[index] = sensor_handle;
  }
}

void CoppeliaSimRobot::forward_engage_wheel_motors(bool value) {
  int v = value ? 1 : 0;

  for (size_t i = 0; i < wheel_joint_handles.size; i++) {
    spdlog::info("[Chassis] Set joint {} motor enabled to {}", wheel_joint_handles[i], v);
    simSetObjectInt32Param(wheel_joint_handles[i], sim_jointintparam_motor_enabled, v);
  }
}
