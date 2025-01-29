#include "coppeliasim_robot.hpp"

#include <algorithm>

// #define TIME_VISION

#ifdef TIME_VISION
#include <chrono>
#endif

#ifdef USE_OPENCV
#include "opencv2/imgproc.hpp"
#endif

#include "spdlog/spdlog.h"

#if SIM_PROGRAM_VERSION_NB >= 40800
bool read_vector_from_signal(int handle, const std::string &name,
                             Vector3 &out) {
  double values[3];
  int r = simGetVector3Property(handle, name.data(), values);
  if (r) {
    out.x = values[0];
    out.y = values[1];
    out.z = values[2];
    return true;
  }
  return false;
}
#else
bool read_vector_from_signal(int, const std::string &name, Vector3 &out) {
  int buffer_lenght;
  char *buffer = simGetStringSignal(name.data(), &buffer_lenght);
  if (buffer && buffer_lenght == sizeof(Vector3)) {
    memcpy(&(out.x), buffer, buffer_lenght);
    return true;
  }
  return false;
}
#endif

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
  spdlog::debug("Set led {} color to {} {} {}", index, color[0], color[1],
                color[2]);
  simSetShapeColor(chassis_led_handles[index], nullptr,
                   sim_colorcomponent_emission, color);
}

void CoppeliaSimRobot::forward_gimbal_led(size_t index, size_t part,
                                          const Color &rgb) {
  if (index >= gimbal_led_handles.size ||
      gimbal_led_handles[index].size() <= part) {
    spdlog::warn("Unknown gimbal led {}[{}]", index, part);
    return;
  }
  float color[3] = {rgb.r, rgb.g, rgb.b};
  spdlog::debug("Set gimbal led {}-{} color to {} {} {} [{}]", index, part,
                color[0], color[1], color[2], gimbal_led_handles[index][part]);
  simSetShapeColor(gimbal_led_handles[index][part], nullptr,
                   sim_colorcomponent_emission, color);
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
    // I need to read a simFloat because from version 4.5, coppeliaSim uses
    // double instead of floats in most apis.
    simFloat v;
    simGetObjectFloatParam(wheel_joint_handles[i], sim_jointfloatparam_velocity,
                           &v);
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
  if (!read_vector_from_signal(gyro_handle, gyro_signal, imu.angular_velocity)) {
    spdlog::warn("Could not read gyro");
    return {};
  }
  if (!read_vector_from_signal(accelerometer_handle, accelerometer_signal, imu.acceleration)) {
    spdlog::warn("Could not read accelerometer");
    return {};
  }
  // TODO(IMU has no attitude->should move it in a separate method)
  simFloat euler_angles[3];
  if (simGetObjectOrientation(imu_handle, -1, euler_angles) < 0) {
    spdlog::warn("Could not get orientation of object {} [imu_handle]",
                 imu_handle);
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

// void CoppeliaSimRobot::update_orientation(float alpha, float beta, float
// gamma) {
//   chassis.attitude.roll = alpha;
//   chassis.attitude.pitch = beta;
// }

std::vector<uint8_t> CoppeliaSimRobot::read_camera_image() const {
  if (!camera_handle)
    return {};
  simHandleVisionSensor(camera_handle, nullptr, nullptr);
  int resolution[2]{};
#if SIM_PROGRAM_VERSION_NB >= 40400
  unsigned char *buffer = simGetVisionSensorImg(camera_handle, 0, 0.0, nullptr,
                                                nullptr, resolution);
#else
  unsigned char *buffer =
      simGetVisionSensorCharImage(handle, &resolution[0], &resolution[1]);
#endif
  const int width = resolution[0];
  const int height = resolution[1];
  if (width != camera.width || height != camera.height) {
    spdlog::warn(
        "Skip frame because of uncorrect size ({}, {}) vs desired size ({}, "
        "{})",
        width, height, camera.width, camera.height);
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

bool CoppeliaSimRobot::forward_camera_resolution(unsigned width,
                                                 unsigned height) {
  if (!camera_handle)
    return false;
  int image_size[2]{};
#if SIM_PROGRAM_VERSION_NB >= 40500
  simGetVisionSensorRes(camera_handle, image_size);
#else
  simGetVisionSensorResolution(camera_handle, image_size);
#endif
  if (width != image_size[0] || height != image_size[1]) {
    simSetObjectInt32Param(camera_handle, sim_visionintparam_resolution_x,
                           width);
    simSetObjectInt32Param(camera_handle, sim_visionintparam_resolution_y,
                           height);
    spdlog::warn("Changing camera resolution from ({}, {}) to ({}, {})",
                 image_size[0], image_size[1], width, height);
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
      simSetObjectInt32Param(servo_handles.at(index),
                             sim_jointintparam_ctrl_enabled, 1);
    } else {
      simSetObjectInt32Param(servo_handles.at(index),
                             sim_jointintparam_ctrl_enabled, 0);
    }
  }
}

void CoppeliaSimRobot::forward_servo_enabled(size_t index, bool value) {
  if (servo_handles.at(index) > 0) {
    simSetObjectInt32Param(servo_handles.at(index),
                           sim_jointintparam_motor_enabled, value);
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
    simGetObjectFloatParam(servo_handles.at(index),
                           sim_jointfloatparam_velocity, &speed);
  }
  return speed;
}

void CoppeliaSimRobot::forward_target_gripper(Gripper::Status state,
                                              float power) {
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

// static cv::Rect find_bounding_box(const cv::Mat & image, const cv::Scalar &
// color) {
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

constexpr std::array<int, 6> size_params{
    sim_objfloatparam_modelbbox_min_x, sim_objfloatparam_modelbbox_max_x,
    sim_objfloatparam_modelbbox_min_y, sim_objfloatparam_modelbbox_max_y,
    sim_objfloatparam_modelbbox_min_z, sim_objfloatparam_modelbbox_max_z};

static std::array<std::array<simFloat, 3>, 8> get_vertices(int handle) {
  std::array<std::array<simFloat, 3>, 8> vertices;
  int index = 0;
  for (int i = 0; i < 2; ++i) {
    simFloat x;
    simGetObjectFloatParam(handle, size_params[i], &x);
    for (int j = 2; j < 4; ++j) {
      simFloat y;
      simGetObjectFloatParam(handle, size_params[j], &y);
      for (int k = 4; k < 6; ++k) {
        simFloat z;
        simGetObjectFloatParam(handle, size_params[k], &z);
        vertices[index++] = {x, y, z};
      }
    }
  }
  return vertices;
}

void CoppeliaSimRobot::set_vision_class(const std::string &name, uint8_t kind) {
  const std::string fname = "/" + name;
  for (int i = 0;; i++) {
    int root = simGetObject(fname.c_str(), i, -1, 1);
    if (root == -1)
      break;
    if (root == root_handle)
      continue;
    vertices_for_root[root] = get_vertices(root);
    int count;
    int *hs = simGetObjectsInTree(root, sim_object_shape_type, 0, &count);
    for (int j = 0; j < count; ++j) {
      vision_class_for_handle[hs[j]] = kind;
      root_for_handle[hs[j]] = root;
    }
    simReleaseBuffer((const char *)hs);
  }
}

BoundingBox CoppeliaSimRobot::project_model(int handle, int camera_handle,
                                            float image_ratio) const {
  simFloat m[12];
  simGetObjectMatrix(handle, camera_handle, m);
  float min_x = 1e3;
  float max_x = -1e3;
  float min_y = 1e3;
  float max_y = -1e3;
  for (const auto &vertex : vertices_for_root.at(handle)) {
    auto c_vertex = vertex;
    simTransformVector(m, c_vertex.data());
    const float mx = c_vertex[2] * tan_fov_2;
    const float my = mx / image_ratio;
    const float x = c_vertex[0] / mx * 0.5f;
    const float y = c_vertex[1] / my * 0.5f;
    min_x = std::min(min_x, x);
    min_y = std::min(min_y, y);
    max_x = std::max(max_x, x);
    max_y = std::max(max_y, y);
  }
  return {0.5f - 0.5f * (max_x + min_x), 0.5f - 0.5f * (max_y + min_y),
          max_x - min_x, max_y - min_y};
}

static bool should_detect(const BoundingBox &actual, const BoundingBox &model,
                          float e, float min_width, float min_height) {
  spdlog::info("should_detect {} {} | {} {} {}", actual, model, e, min_width,
               min_height);
  return ((actual.width + e) > model.width * min_width) &&
         ((actual.height + e) > model.height * min_height);
}

// 0.02 ms
#ifndef USE_OPENCV

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
    return BoundingBox((float)(max_j + min_j) / w * 0.5,
                       1.0 - (float)(max_i + min_i) / h * 0.5,
                       (float)(max_j - min_j + 1) / w,
                       (float)(max_i - min_i + 1) / h);
  }
};

// {class -> {root -> BoundingBox}}
std::map<uint8_t, std::map<int, BoundingBox>>
CoppeliaSimRobot::detect_bounding_boxes(const unsigned char *buffer, int width,
                                        int height, uint8_t mask) const {
  // {class -> {root -> Rect}}
  std::map<uint8_t, std::map<int, ImageRect>> rects;
  for (uint8_t k = 0; k < 8; ++k) {
    if ((1 << k) & mask) {
      rects[k] = {};
    }
  }
  for (size_t i = 0; i < height; i++) {
    for (size_t j = 0; j < width; j++) {
      const int handle = buffer[0] + (buffer[1] << 8) + (buffer[2] << 16);
      if (vision_class_for_handle.count(handle)) {
        const uint8_t k = vision_class_for_handle.at(handle);
        const uint8_t r = root_for_handle.at(handle);
        if ((1 << k) & mask) {
          auto box = rects[k].find(r);
          if (box == rects[k].end()) {
            rects[k].emplace(std::piecewise_construct, std::make_tuple(r),
                             std::make_tuple(i, j));
          } else {
            box->second.update(i, j);
          }
        }
      }
      buffer += 3;
    }
  }
  std::map<uint8_t, std::map<int, BoundingBox>> boxes;
  for (const auto &[k, vs] : rects) {
    boxes[k] = {};
    for (const auto &[h, v] : vs) {
      boxes[k].emplace(h, v.box(width, height));
    }
  }
  return boxes;
}

#else

static BoundingBox box(const cv::Mat &mat, uint16_t w, uint16_t h) {
  cv::Mat points;
  cv::findNonZero(mat, points);
  const cv::Rect bb = cv::boundingRect(points);
  return BoundingBox((float)(bb.x + bb.width * 0.5) / w,
                     1.0 - (float)(bb.y + bb.height * 0.5) / h,
                     (float)(bb.width + 1) / w, (float)(bb.height + 1) / h);
}

// 0.04 ms
// {class -> {root -> BoundingBox}}
std::map<uint8_t, std::map<int, BoundingBox>>
CoppeliaSimRobot::detect_bounding_boxes(const unsigned char *buffer, int width,
                                        int height, uint8_t mask) const {
  std::map<uint8_t, std::map<int, cv::Mat>> mask_images;
  for (uint8_t k = 0; k < 8; ++k) {
    if ((1 << k) & mask) {
      mask_images[k] = {};
    }
  }
  for (size_t i = 0; i < height; i++) {
    for (size_t j = 0; j < width; j++) {
      const int handle = buffer[0] + (buffer[1] << 8) + (buffer[2] << 16);
      if (vision_class_for_handle.count(handle)) {
        const uint8_t k = vision_class_for_handle.at(handle);
        if ((1 << k) & mask) {
          const uint8_t r = root_for_handle.at(handle);
          if (!mask_images[k].count(r)) {
            mask_images[k][r] = cv::Mat::zeros(height, width, CV_8U);
          }
          mask_images[k].at(r).at<unsigned char>(i, j) = 255;
        }
      }
      buffer += 3;
    }
  }
  std::map<uint8_t, std::map<int, BoundingBox>> boxes;
  for (auto &[k, vs] : mask_images) {
    boxes[k] = {};
    for (auto &[h, v] : vs) {
      boxes[k].emplace(h, box(v, width, height));
    }
  }
  return boxes;
}

#endif

DetectedObjects CoppeliaSimRobot::run_detector(const unsigned char *buffer,
                                               int width, int height,
                                               uint8_t mask) const {
  // {class -> {root -> BoundingBox}}
  auto boxes = detect_bounding_boxes(buffer, width, height, mask);
  DetectedObjects objects;
  // const float detection_tolerance = 2.0f / std::min(width, height);
  const float image_ratio =
      static_cast<float>(width) / static_cast<float>(height);
  for (const auto &[k, vs] : boxes) {
    if (k == DetectedObjects::Type::PERSON) {
      for (const auto &[h, b] : vs) {
        const auto bm = project_model(h, vision_handle, image_ratio);
        if (should_detect(b, bm, detection_tolerance, min_detection_width,
                          min_detection_height)) {
          objects.people.emplace_back(b, h);
          // spdlog::info("detected person {} {}", h, b);
        }
      }
    }
    if (k == DetectedObjects::Type::ROBOT) {
      for (const auto &[h, b] : vs) {
        const auto bm = project_model(h, vision_handle, image_ratio);
        if (should_detect(b, bm, detection_tolerance, min_detection_width,
                          min_detection_height)) {
          objects.robots.emplace_back(b, h);
          // spdlog::info("detected robot {} {}", h, b);
        }
      }
    }
  }
  return objects;
}

// Basic version is taking
// d1 2ms, d2 0.02 ms
// OpenCV version is taking
// d2 0.04 ms

DetectedObjects CoppeliaSimRobot::read_detected_objects() const {
  if (vision_handle <= 0) {
    spdlog::warn("No vision sensor");
    return {};
  }
#ifdef TIME_VISION
  auto start = std::chrono::steady_clock::now();
#endif
  simHandleVisionSensor(vision_handle, nullptr, nullptr);
  int resolution[2]{};
#if SIM_PROGRAM_VERSION_NB >= 40400
  unsigned char *buffer = simGetVisionSensorImg(vision_handle, 0, 0.0, nullptr,
                                                nullptr, resolution);
#else
  unsigned char *buffer = simGetVisionSensorCharImage(
      vision_handle, &resolution[0], &resolution[1]);
#endif
  const int width = resolution[0];
  const int height = resolution[1];
#ifdef TIME_VISION
  auto middle = std::chrono::steady_clock::now();
#endif
  auto rs = run_detector(buffer, width, height, vision.get_enable());
  simReleaseBuffer((const char *)buffer);
#ifdef TIME_VISION
  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<double> d2 = end - middle;
  std::chrono::duration<double> d1 = middle - start;
  spdlog::info("Vision: read image {:.0f} us, detect boxes {:.0f} us",
               1e6 * d1.count(), 1e6 * d2.count());
#endif
  return rs;
}

void CoppeliaSimRobot::init_vision() {
  if (vision_handle > 0) {
    simFloat value;
    simGetObjectFloatParam(vision_handle,
                           sim_visionfloatparam_perspective_angle, &value);
    tan_fov_2 = tan(value * 0.5);
    set_vision_class("RoboMaster", DetectedObjects::Type::ROBOT);
    set_vision_class("Bill", DetectedObjects::Type::PERSON);
  }
}

void CoppeliaSimRobot::configure_vision(float min_width, float min_height,
                                        float tolerance) {
  min_detection_width = std::max(min_width, 0.0f);
  min_detection_height = std::max(min_height, 0.0f);
  detection_tolerance = std::max(tolerance, 0.0f);
}

hit_event_t CoppeliaSimRobot::read_hit_events() const { return {}; }

float CoppeliaSimRobot::read_tof(size_t index) const {
  if (!tof_handles.count(index))
    return 0.0f;
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
    spdlog::info("[Chassis] Set joint {} motor enabled to {}",
                 wheel_joint_handles[i], v);
    simSetObjectInt32Param(wheel_joint_handles[i],
                           sim_jointintparam_motor_enabled, v);
  }
}
