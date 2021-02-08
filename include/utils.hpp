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
    float s = sin(theta);
    float c = cos(theta);
    return {x * c - y * s, x * s + y * c, z};
  }

  template <typename OStream> friend OStream &operator<<(OStream &os, const Vector3 &r) {
    os << "Vector3 < " << r.x << ", " << r.y << ", " << r.z << " >";
    return os;
  }

  inline Vector3 operator*(float v) const { return {x * v, y * v, z * v}; }

  inline Vector3 operator/(float v) const { return {x / v, y / v, z / v}; }

  inline Vector3 operator+(Vector3 v) const { return {x + v.x, y + v.y, z + v.z}; }

  inline Vector3 operator-(Vector3 v) const { return {x - v.x, y - v.y, z - v.z}; }

  inline float norm() const { return sqrt(x * x + y * y + z * z); }
};

struct Twist2D {
  float x, y, theta;

  inline Twist2D operator*(float f) const { return {x * f, y * f, theta * f}; }

  Twist2D rotate_around_z(float alpha) const {
    Vector3 p{x, y, 0};
    p = p.rotate_around_z(alpha);
    return {p.x, p.y, theta};
  }

  template <typename OStream> friend OStream &operator<<(OStream &os, const Twist2D &r) {
    os << "Twist2D < " << r.x << ", " << r.y << ", " << r.theta << " >";
    return os;
  }
};

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

  inline float norm() const { return distance() + abs(normalize(theta)); }

  // Pose2D relative_to(Pose2D pose) {
  //   Twist2D delta = *this - pose;
  //   delta = delta.rotate_around_z(-pose.theta);
  //   return {delta.x, delta.y, delta.theta};
  // }

  // p^-1 * this
  Pose2D relative_to(Pose2D pose) { return pose.inverse() * (*this); }

  template <typename OStream> friend OStream &operator<<(OStream &os, const Pose2D &r) {
    os << std::setprecision(3);
    os << "Pose2D < " << r.x << ", " << r.y << ", " << r.theta << " >";
    return os;
  }
};

template <typename T> struct WheelValues {
  T front_left;
  T front_right;
  T rear_left;
  T rear_right;

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
  os << "<"
     << " front left: " << v.front_left << ", front right: " << v.front_right
     << ", rear left: " << v.rear_left << ", rear right: " << v.rear_right << " >";
  return os;
}

template <typename T> struct LEDValues {
  T front;
  T left;
  T rear;
  T right;

  inline bool operator==(const LEDValues<T> &rhs) const {
    return front == rhs.front && left == rhs.left && rear == rhs.rear && right == rhs.right;
  }

  inline bool operator!=(const LEDValues<T> &rhs) { return !(*this == rhs); }

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

template <typename OStream, typename T> OStream &operator<<(OStream &os, const LEDValues<T> &v) {
  os << "<"
     << "front: " << v.front << ", left: " << v.left << ", rear: " << v.rear
     << ", right: " << v.right << " >";
  return os;
}

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

template <typename T> struct ServoValues {
  T right;
  T left;

  inline bool operator==(const ServoValues<T> &rhs) const {
    return right == rhs.right && left == rhs.left;
  }

  inline bool operator!=(const ServoValues<T> &rhs) const { return !(*this == rhs); }

  inline ServoValues<T> operator+(const ServoValues<T> &rhs) const {
    return {right + rhs.right, left + rhs.left};
  }

  inline ServoValues<T> operator-(const ServoValues<T> &rhs) const {
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

template <typename OStream, typename T> OStream &operator<<(OStream &os, const ServoValues<T> &v) {
  os << "{"
     << "\n\rright: " << v.right << "\n\rleft: " << v.left << "\n}";
  return os;
}

struct Vector2 {
  float x, y;

  Vector2 rotate_around_z(float theta) {
    float s = sin(theta);
    float c = cos(theta);
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
};

// The values at the RM reset position (maximal flexion)
constexpr ServoValues<float> SERVO_RESET_ANGLES = {.right = -0.274016f, .left = 0.073304f};
constexpr ServoValues<int> SERVO_RESET_VALUES = {.right = 1273, .left = 1242};
constexpr float SERVO_RAD2UNIT = 325.95f;
constexpr float SERVO_RAD_PER_SECOND2UNIT = 325.95f * 5.0f;
constexpr ServoValues<float> SERVO_RESET_EXT_ANGLES = {
    .right = static_cast<float>(SERVO_RESET_VALUES.right / SERVO_RAD2UNIT),
    .left = static_cast<float>(SERVO_RESET_VALUES.left / SERVO_RAD2UNIT)};

constexpr ServoValues<float> SERVO_BIASES = {
    .right = SERVO_RESET_VALUES.right + SERVO_RAD2UNIT * SERVO_RESET_ANGLES.right,
    .left = SERVO_RESET_VALUES.left + SERVO_RAD2UNIT * SERVO_RESET_ANGLES.left};

inline int servo_speed_value(float speed) {
  return static_cast<int>(round(-SERVO_RAD_PER_SECOND2UNIT * speed));
}

inline int servo_angle_value(size_t index, float angle) {
  return static_cast<int>(round(-SERVO_RAD2UNIT * angle + SERVO_BIASES[index]));
}

#endif  // INCLUDE_UTILS_HPP_
