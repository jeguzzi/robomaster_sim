#ifndef INCLUDE_UTILS_HPP_
#define INCLUDE_UTILS_HPP_

#define _USE_MATH_DEFINES
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"
#include <fmt/ostream.h>
#include <fmt/core.h>

#if FMT_VERSION >= 90000 
#define STREAM(X) fmt::streamed(X)
#else
#define STREAM(X) (X)
#endif

enum ValueType { CURRENT, TARGET, DESIRED };

template <typename T> struct ControllableValue {
  T current;
  T target;
  T desired;
  bool initialized;

  bool check() {
    if (target != desired || !initialized) {
      target = desired;
      initialized = true;
      return true;
    }
    return false;
  }

  explicit ControllableValue(T value)
      : current(value)
      , target(value)
      , desired(value)
      , initialized(false) {}

  void set_target(const T &value) { desired = value; }

  void set(const T &value, ValueType type = CURRENT) {
    switch (type) {
    case CURRENT:
      current = value;
      return;
    case DESIRED:
      desired = value;
      return;
    case TARGET:
      target = value;
      return;
    }
  }

  T get(ValueType type) {
    switch (type) {
    case CURRENT:
      return current;
    case DESIRED:
      return desired;
    case TARGET:
      return target;
    }
    return current;
  }
};

// float normalize(float value);

inline float normalize(float value) {
  value = fmod(value + M_PI, 2 * M_PI);
  if (value < 0)
    value += 2 * M_PI;
  return value - M_PI;
}

struct Vector3 {
  float x, y, z;

  Vector3 rotate_around_z(float theta) const {
    float s = std::sin(theta);
    float c = std::cos(theta);
    return {x * c - y * s, x * s + y * c, z};
  }

  template <typename OStream> friend OStream &operator<<(OStream &os, const Vector3 &r) {
    os << std::fixed << std::setprecision(3) << "Vector3 < " << r.x << ", " << r.y << ", " << r.z
       << " >";
    return os;
  }

  inline Vector3 operator*(float v) const { return {x * v, y * v, z * v}; }

  inline Vector3 operator/(float v) const { return {x / v, y / v, z / v}; }

  inline Vector3 operator+(Vector3 v) const { return {x + v.x, y + v.y, z + v.z}; }

  inline Vector3 operator-(Vector3 v) const { return {x - v.x, y - v.y, z - v.z}; }

  inline float norm() const { return sqrt(x * x + y * y + z * z); }
};

#if FMT_VERSION >= 90000 
  template <> struct fmt::formatter<Vector3> : ostream_formatter {};
#endif

template <typename T> struct GimbalValues;

struct Attitude {
  float yaw, pitch, roll;

  template <typename OStream> friend OStream &operator<<(OStream &os, const Attitude &v) {
    os << std::fixed << std::setprecision(3) << "Attitude < " << v.roll << ", " << v.pitch << ", "
       << v.yaw << " >";
    return os;
  }

  inline Attitude operator+(Attitude v) const {
    return {yaw + v.yaw, pitch + v.pitch, roll + v.roll};
  }

  operator GimbalValues<float>() const;
};

#if FMT_VERSION >= 90000 
  template <> struct fmt::formatter<Attitude> : ostream_formatter {};
#endif

struct Twist2D {
  float x, y, theta;

  inline Twist2D operator*(float f) const { return {x * f, y * f, theta * f}; }

  Twist2D rotate_around_z(float alpha) const {
    Vector3 p{x, y, 0};
    p = p.rotate_around_z(alpha);
    return {p.x, p.y, theta};
  }

  template <typename OStream> friend OStream &operator<<(OStream &os, const Twist2D &r) {
    os << std::fixed << std::setprecision(3) << "Twist2D < " << r.x << ", " << r.y << ", "
       << r.theta << " >";
    return os;
  }
};

#if FMT_VERSION >= 90000 
  template <> struct fmt::formatter<Twist2D> : ostream_formatter {};
#endif

struct Pose2D {
  float x, y, theta;

  // inline Pose2D operator+(Twist2D & v) {
  //   return {x + v.x, y + v.y, normalize(theta + v.theta)};
  // }

  inline Pose2D operator+(Twist2D v) const { return {x + v.x, y + v.y, theta + v.theta}; }

  inline Twist2D operator-(Pose2D p) const {
    return {x + p.x, y + p.y, normalize(theta - p.theta)};
  }

  Pose2D rotate_around_z(float alpha) const {
    Vector3 p{x, y, 0};
    p = p.rotate_around_z(alpha);
    return {p.x, p.y, theta + alpha};
  }

  // Composition of this and p, i.e.,
  // when interpreting p as a transormation: first apply p than this
  // inline Pose2D operator*(Pose2D p) {
  //   float s = sin(theta);
  //   float c = cos(theta);
  //   return {p.x * c - p.y * s + x, p.x * s + p.y * c + y, p.theta + theta};
  // }
  //
  inline Pose2D operator*(Pose2D p) const {
    Pose2D q = p.rotate_around_z(theta);
    return {q.x + x, q.y + y, p.theta + theta};
  }

  inline Pose2D inverse() const {
    Pose2D q = rotate_around_z(-theta);
    return {-q.x, -q.y, -theta};
  }

  inline float distance() const { return sqrt(x * x + y * y); }

  inline float norm() const { return distance() + std::abs(normalize(theta)); }

  // Pose2D relative_to(Pose2D pose) {
  //   Twist2D delta = *this - pose;
  //   delta = delta.rotate_around_z(-pose.theta);
  //   return {delta.x, delta.y, delta.theta};
  // }

  // p^-1 * this
  Pose2D relative_to(Pose2D pose) { return pose.inverse() * (*this); }

  template <typename OStream> friend OStream &operator<<(OStream &os, const Pose2D &r) {
    os << std::fixed << std::setprecision(3);
    os << "Pose2D < " << r.x << ", " << r.y << ", " << r.theta << " >";
    return os;
  }
};

#if FMT_VERSION >= 90000 
  template <> struct fmt::formatter<Pose2D> : ostream_formatter {};
#endif

template <typename T> struct WheelValues {
  T front_left;
  T front_right;
  T rear_left;
  T rear_right;

  static constexpr size_t size = 4;

  inline bool operator==(const WheelValues<T> &rhs) const {
    return front_left == rhs.front_left && front_right == rhs.front_right &&
           rear_left == rhs.rear_left && rear_right == rhs.rear_right;
  }

  inline bool operator!=(const WheelValues<T> &rhs) { return !(*this == rhs); }

  inline WheelValues<T> operator*(const T &rhs) {
    return {front_left * rhs, front_right * rhs, rear_left * rhs, rear_right * rhs};
  }

  inline WheelValues<T> operator+(const WheelValues<T> &rhs) {
    return {front_left + rhs.front_left, front_right + rhs.front_right, rear_left + rhs.rear_left,
            rear_right + rhs.rear_right};
  }

  T &operator[](std::size_t idx) {
    switch (idx) {
    case 0:
      return front_left;
    case 1:
      return front_right;
    case 2:
      return rear_left;
    case 3:
      return rear_right;
    default:
      throw std::out_of_range("Invalid position!");
    }
  }
  const T &operator[](std::size_t idx) const {
    switch (idx) {
    case 0:
      return front_left;
    case 1:
      return front_right;
    case 2:
      return rear_left;
    case 3:
      return rear_right;
    default:
      throw std::out_of_range("Invalid position!");
    }
  }
};

template <typename OStream, typename T> OStream &operator<<(OStream &os, const WheelValues<T> &v) {
  os << std::fixed << std::setprecision(3);
  os << "Wheel < "
     << " front left: " << v.front_left << ", front right: " << v.front_right
     << ", rear left: " << v.rear_left << ", rear right: " << v.rear_right << " >";
  return os;
}

#if FMT_VERSION >= 90000 
  template <typename T> struct fmt::formatter<WheelValues<T>> : ostream_formatter {};
#endif

template <typename T> struct ChassisLEDValues {
  T front;
  T left;
  T rear;
  T right;

  static constexpr size_t size = 4;

  inline bool operator==(const ChassisLEDValues<T> &rhs) const {
    return front == rhs.front && left == rhs.left && rear == rhs.rear && right == rhs.right;
  }

  inline bool operator!=(const ChassisLEDValues<T> &rhs) { return !(*this == rhs); }

  T &operator[](std::size_t idx) {
    switch (idx) {
    case 0:
      return front;
    case 1:
      return left;
    case 2:
      return rear;
    case 3:
      return right;
    default:
      throw std::out_of_range("Invalid position!");
    }
  }
  const T &operator[](std::size_t idx) const {
    switch (idx) {
    case 0:
      return front;
    case 1:
      return left;
    case 2:
      return rear;
    case 3:
      return right;
    default:
      throw std::out_of_range("Invalid position!");
    }
  }
};

template <typename OStream, typename T>
OStream &operator<<(OStream &os, const ChassisLEDValues<T> &v) {
  os << "ChassisLED < "
     << "front: " << v.front << ", left: " << v.left << ", rear: " << v.rear
     << ", right: " << v.right << " >";
  return os;
}

#if FMT_VERSION >= 90000 
  template <typename T> struct fmt::formatter<ChassisLEDValues<T>> : ostream_formatter {};
#endif

template <typename T> struct GimbalLEDValues {
  T top_left;
  T top_right;

  constexpr static size_t size = 2;

  inline bool operator==(const GimbalLEDValues<T> &rhs) const {
    return top_right == rhs.top_right && top_left == rhs.top_left;
  }

  inline bool operator!=(const GimbalLEDValues<T> &rhs) { return !(*this == rhs); }

  T &operator[](std::size_t idx) {
    switch (idx) {
    case 0:
      return top_left;
    case 1:
      return top_right;
    default:
      throw std::out_of_range("Invalid position!");
    }
  }
  const T &operator[](std::size_t idx) const {
    switch (idx) {
    case 0:
      return top_left;
    case 1:
      return top_right;
    default:
      throw std::out_of_range("Invalid position!");
    }
  }
};

template <typename OStream, typename T>
OStream &operator<<(OStream &os, const GimbalLEDValues<T> &v) {
  os << "GimbalLED < "
     << "top left: " << v.top_left << ", top right: " << v.top_right << " >";
  return os;
}

#if FMT_VERSION >= 90000 
  template <typename T> struct fmt::formatter<GimbalLEDValues<T>> : ostream_formatter {};
#endif

inline unsigned key_from(uint8_t _set, uint8_t _cmd) { return _cmd + 256 * _set; }

constexpr float angular_speed_from_rpm(int rpm) { return 2 * M_PI * rpm / 60.0; }

inline int rpm_from_angular_speed(int speed) {
  return static_cast<int>(round(speed * 60.0 / (2 * M_PI)));
}

constexpr float deg2rad(float value) { return M_PI * value / 180.0; }

constexpr float rad2deg(float value) { return 180.0 * value * M_1_PI; }

template <typename T> T read(const uint8_t *buffer) {
  T value;
  uint8_t *bytes = reinterpret_cast<uint8_t *>(&value);
  memcpy(bytes, buffer, sizeof(T));
  return value;
}

template <typename T> void write(std::vector<uint8_t> &buffer, size_t index, T value) {
  uint8_t *bytes = reinterpret_cast<uint8_t *>(&value);
  for (size_t i = 0; i < sizeof(T); i++) {
    buffer[index + i] = bytes[i];
  }
}

template <class T> using ServoValues = std::array<T, 3>;

template <typename OStream, typename T> OStream &operator<<(OStream &os, const ServoValues<T> &v) {
  os << "Servos < ";
  for (size_t i = 0; i < v.size(); i++) {
    os << "#" << i << ": " << v[i] << ", ";
  }
  os << " >";
  return os;
}

#if FMT_VERSION >= 90000 
  template <typename T> struct fmt::formatter<ServoValues<T>> : ostream_formatter {};
#endif

template <typename T> struct ArmServoValues {
  T right;
  T left;

  inline bool operator==(const ArmServoValues<T> &rhs) const {
    return right == rhs.right && left == rhs.left;
  }

  inline bool operator!=(const ArmServoValues<T> &rhs) const { return !(*this == rhs); }

  inline ArmServoValues<T> operator+(const ArmServoValues<T> &rhs) const {
    return {right + rhs.right, left + rhs.left};
  }

  inline ArmServoValues<T> operator-(const ArmServoValues<T> &rhs) const {
    return {right - rhs.right, left - rhs.left};
  }

  T &operator[](std::size_t idx) {
    switch (idx) {
    case 0:
      return left;
    case 1:
      return right;
    default:
      throw std::out_of_range("Invalid position!");
    }
  }
  const T &operator[](std::size_t idx) const {
    switch (idx) {
    case 0:
      return left;
    case 1:
      return right;
    default:
      throw std::out_of_range("Invalid position!");
    }
  }

  operator bool() const { return static_cast<bool>(right) && static_cast<bool>(left); }
};

template <typename OStream, typename T>
OStream &operator<<(OStream &os, const ArmServoValues<T> &v) {
  os << std::fixed << std::setprecision(3);
  os << "ArmServo < "
     << "left: " << v.left << ", right: " << v.right << " >";
  return os;
}

#if FMT_VERSION >= 90000 
  template <typename T> struct fmt::formatter<ArmServoValues<T>> : ostream_formatter {};
#endif


template <typename T> struct GimbalValues {
  T yaw;
  T pitch;

  static constexpr size_t size = 2;

  inline bool operator==(const GimbalValues<T> &rhs) const {
    return yaw == rhs.yaw && pitch == rhs.pitch;
  }

  inline bool operator!=(const GimbalValues<T> &rhs) const { return !(*this == rhs); }

  inline GimbalValues<T> operator*(float rhs) const { return {yaw * rhs, pitch * rhs}; }

  inline GimbalValues<T> operator+(const GimbalValues<T> &rhs) const {
    return {yaw + rhs.yaw, pitch + rhs.pitch};
  }

  inline GimbalValues<T> operator-(const GimbalValues<T> &rhs) const {
    return {yaw - rhs.yaw, pitch - rhs.pitch};
  }

  T &operator[](std::size_t idx) {
    switch (idx) {
    case 0:
      return yaw;
    case 1:
      return pitch;
    default:
      throw std::out_of_range("Invalid position!");
    }
  }
  const T &operator[](std::size_t idx) const {
    switch (idx) {
    case 0:
      return yaw;
    case 1:
      return pitch;
    default:
      throw std::out_of_range("Invalid position!");
    }
  }

  operator bool() const { return static_cast<bool>(yaw) && static_cast<bool>(pitch); }

  operator Attitude() const { return {.yaw = yaw, .pitch = pitch, .roll = 0}; }
};

template <typename OStream, typename T> OStream &operator<<(OStream &os, const GimbalValues<T> &v) {
  os << "Gimbal < "
     << "yaw: " << v.yaw << ", pitch :" << v.pitch << " >";
  return os;
}


#if FMT_VERSION >= 90000 
  template <typename T> struct fmt::formatter<GimbalValues<T>> : ostream_formatter {};
#endif

inline Attitude::operator GimbalValues<float>() const { return {.yaw = yaw, .pitch = pitch}; }

struct Vector2 {
  float x, y;

  Vector2 rotate_around_z(float theta) {
    float s = std::sin(theta);
    float c = std::cos(theta);
    return {x * c - y * s, x * s + y * c};
  }

  template <typename OStream> friend OStream &operator<<(OStream &os, const Vector2 &r) {
    os << "Vector2 < " << r.x << ", " << r.y << " >";
    return os;
  }

  inline Vector2 operator*(float v) { return {x * v, y * v}; }

  inline Vector2 operator/(float v) { return {x / v, y / v}; }

  inline Vector2 operator+(Vector2 v) { return {x + v.x, y + v.y}; }

  inline Vector2 operator-(Vector2 v) { return {x - v.x, y - v.y}; }

  inline float norm() { return sqrt(x * x + y * y); }

  float &operator[](std::size_t idx) {
    switch (idx) {
    case 0:
      return x;
    case 1:
      return y;
    default:
      throw std::out_of_range("Invalid position!");
    }
  }

  const float &operator[](std::size_t idx) const {
    switch (idx) {
    case 0:
      return x;
    case 1:
      return y;
    default:
      throw std::out_of_range("Invalid position!");
    }
  }
};

#if FMT_VERSION >= 90000 
  template <> struct fmt::formatter<Vector2> : ostream_formatter {};
#endif

struct Matrix2 {
  float values[2][2];

  Matrix2(float a, float b, float c, float d) {
    values[0][0] = a;
    values[0][1] = b;
    values[1][0] = c;
    values[1][1] = d;
  }

  inline float det() { return values[0][0] * values[1][1] - values[0][1] * values[1][0]; }

  inline Matrix2 inverse() {
    float d = det();
    return {values[1][1] / d, -values[0][1] / d, -values[1][0] / d, values[0][0] / d};
  }

  inline Vector2 operator*(const Vector2 &v) {
    Vector2 r;
    for (size_t i = 0; i < 2; i++) {
      r[i] = 0;
      for (size_t j = 0; j < 2; j++) {
        r[i] += v[j] * values[i][j];
      }
    }
    return r;
  }

  template <typename OStream> friend OStream &operator<<(OStream &os, const Matrix2 &r) {
    os << "Matrix2 [[" << r.values[0][0] << ", " << r.values[0][1] << "], "
       << "[" << r.values[1][0] << ", " << r.values[1][1] << "]]";
    return os;
  }
};

#if FMT_VERSION >= 90000 
  template <> struct fmt::formatter<Matrix2> : ostream_formatter {};
#endif

struct BoundingBox {
  float x;
  float y;
  float width;
  float height;
  BoundingBox(float x, float y, float width, float height)
      : x(x)
      , y(y)
      , width(width)
      , height(height) {}

  // BoundingBox(const BoundingBox & other) = default;

  template <typename OStream> friend OStream &operator<<(OStream &os, const BoundingBox &value) {
    os << std::fixed << std::setprecision(3) << "BoundingBox <" << value.x 
       << ", " << value.y << ", " << value.width << ", " << value.height
       << " >";
    return os;
  }
};

#if FMT_VERSION >= 90000 
  template <> struct fmt::formatter<BoundingBox> : ostream_formatter {};
#endif

// The values at the RM reset position (maximal flexion)
constexpr ArmServoValues<float> SERVO_RESET_ANGLES = {.right = -0.274016f, .left = 0.073304f};
// constexpr ArmServoValues<int> SERVO_RESET_VALUES = {.right = 1273, .left = 1242};
// CHANGED(Jerome): 1024 is the value set when adding/connecting the servos.
// i.e., we get the same values on the real robot when adding servos
// in the fully retracted position (SERVO_RESET_ANGLES)
constexpr ArmServoValues<int> SERVO_RESET_VALUES = {.right = 1024, .left = 1024};
// HACK(Jerome): the left an right servos are spinning in opposite direction.
// It would be better to manage it in the robot controller and in Coppelia.
constexpr ArmServoValues<int> SERVO_DIRECTION = {.right = -1, .left = 1};
// 1024 = /pi
constexpr float SERVO_RAD2UNIT = 1024 * M_1_PI;
// rmp / 2
//
constexpr float SERVO_RAD_PER_SECOND2UNIT = SERVO_RAD2UNIT * 5.0f;
constexpr ArmServoValues<float> SERVO_RESET_EXT_ANGLES = {
    .right = static_cast<float>(SERVO_RESET_VALUES.right / SERVO_RAD2UNIT),
    .left = static_cast<float>(SERVO_RESET_VALUES.left / SERVO_RAD2UNIT)};

constexpr ArmServoValues<float> SERVO_BIASES = {
    .right = SERVO_RESET_VALUES.right -
             SERVO_DIRECTION.right * SERVO_RAD2UNIT * SERVO_RESET_ANGLES.right,
    .left = SERVO_RESET_VALUES.left -
            SERVO_DIRECTION.left * SERVO_RAD2UNIT * SERVO_RESET_ANGLES.left};

inline int servo_speed_value(size_t index, float speed) {
  return static_cast<int>(round(SERVO_RAD_PER_SECOND2UNIT * speed * SERVO_DIRECTION[index]));
}

inline int servo_angle_value(size_t index, float angle) {
  return static_cast<int>(
      round(SERVO_DIRECTION[index] * SERVO_RAD2UNIT * angle + SERVO_BIASES[index]));
}

// From absolute rad to servo deg
inline float servo_angle(size_t index, float angle) {
  return rad2deg(SERVO_DIRECTION[index] * (angle - SERVO_RESET_ANGLES[index]));
}

// From zero deg to absolute rad, i.e. the inverse of servo_angle
inline float angle_to_servo(size_t index, float zero_angle) {
  return SERVO_DIRECTION[index] * deg2rad(zero_angle) + SERVO_RESET_ANGLES[index];
}

struct IMU {
  Attitude attitude;
  Vector3 angular_velocity;
  Vector3 acceleration;

  template <typename OStream> friend OStream &operator<<(OStream &os, const IMU &v) {
    os << "IMU < attitude: " << v.attitude << ", angular velocity: " << v.angular_velocity
       << ", acceleration: " << v.acceleration << " >";
    return os;
  }
};

#if FMT_VERSION >= 90000 
  template <> struct fmt::formatter<IMU> : ostream_formatter {};
#endif

struct Odometry {
  Pose2D pose;
  Twist2D twist;

  template <typename OStream> friend OStream &operator<<(OStream &os, const Odometry &v) {
    os << "Odom < pose: " << v.pose << ", twist: " << v.twist << " >";
    return os;
  }
};

#if FMT_VERSION >= 90000 
  template <> struct fmt::formatter<Odometry> : ostream_formatter {};
#endif

#endif  // INCLUDE_UTILS_HPP_
