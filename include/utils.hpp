#ifndef UTILS_HPP
#define UTILS_HPP

#define _USE_MATH_DEFINES
#include <cmath>
#include <string>
#include <vector>
#include <cstdint>
#include <iostream>

#include <spdlog/spdlog.h>
#include "spdlog/fmt/ostr.h"

float normalize(float value);

struct Vector3 {
  float x, y, z;

  Vector3 rotate_around_z(float theta) {
    float s = sin(theta);
    float c = cos(theta);
    return {x * c - y * s, x * s + y * c, z};
  }

  template<typename OStream>
  friend OStream& operator<<(OStream& os, const Vector3& r)
  {
    os << "Vector3 < " << r.x << ", " << r.y << ", "<< r.z << " >";
    return os;
  }

};

struct Twist2D {
  float x, y, theta;

  inline Twist2D operator*(float f) {
    return {x * f, y * f, theta * f};
  }

  Twist2D rotate_around_z(float alpha) {
    Vector3 p {x, y, 0};
    p = p.rotate_around_z(alpha);
    return {p.x, p.y, theta};
  }

  template<typename OStream>
  friend OStream& operator<<(OStream& os, const Twist2D& r)
  {
    os << "Twist2D < " << r.x << ", " << r.y << ", "<< r.theta << " >";
    return os;
  }

};

struct Pose2D {
  float x, y, theta;

  // inline Pose2D operator+(Twist2D & v) {
  //   return {x + v.x, y + v.y, normalize(theta + v.theta)};
  // }

  inline Pose2D operator+(Twist2D v) {
    return {x + v.x, y + v.y, theta + v.theta};
  }

  inline Twist2D operator-(Pose2D p) {

    return {x + p.x, y + p.y, normalize(theta - p.theta)};
  }

  Pose2D rotate_around_z(float alpha) {
    Vector3 p {x, y, 0};
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
  inline Pose2D operator*(Pose2D p) {
    Pose2D q = p.rotate_around_z(theta);
    return {q.x + x, q.y + y, p.theta + theta};
  }

  inline Pose2D inverse() {
    Pose2D q = rotate_around_z(-theta);
    return {-q.x, -q.y, -theta};
  }

  inline float distance() {
    return sqrt(x * x + y * y);
  }

  inline float norm() {
    return distance() + abs(normalize(theta));
  }

  // Pose2D relative_to(Pose2D pose) {
  //   Twist2D delta = *this - pose;
  //   delta = delta.rotate_around_z(-pose.theta);
  //   return {delta.x, delta.y, delta.theta};
  // }

  // p^-1 * this
  Pose2D relative_to(Pose2D pose) {
    return pose.inverse() * (*this);
  }

  template<typename OStream>
  friend OStream& operator<<(OStream& os, const Pose2D& r)
  {
    os << "Pose2D < " << r.x << ", " << r.y << ", "<< r.theta << " >";
    return os;
  }

};


template <typename T>
struct WheelValues {
  T front_left;
  T front_right;
  T rear_left;
  T rear_right;

  inline bool operator==(const WheelValues<T>& rhs) const {
    return front_left == rhs.front_left && front_right == rhs.front_right && rear_left == rhs.rear_left && rear_right == rhs.rear_right;
  }

  inline bool operator!=(const WheelValues<T>& rhs) {
    return !(*this == rhs);
  }

  T& operator[](std::size_t idx)       {
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
  const T& operator[](std::size_t idx) const {
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

template<typename OStream, typename T>
OStream& operator<<(OStream& os, const WheelValues<T>& v)
{
  os << "{"
     << "\n\tfront left: " << v.front_left
     << "\n\tfront right: " << v.front_right
     << "\n\trear left: " << v.rear_left
     << "\n\trear right: " << v.rear_right
     << "\n}";
  return os;
}

template<typename T>
struct LEDValues {
  T front;
  T left;
  T rear;
  T right;

  inline bool operator==(const LEDValues<T>& rhs) const {
    return front == rhs.front && left == rhs.left && rear == rhs.rear && right == rhs.right;
  }

  inline bool operator!=(const LEDValues<T>& rhs) {
    return !(*this == rhs);
  }

  T& operator[](std::size_t idx)       {
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
  const T& operator[](std::size_t idx) const {
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

template<typename OStream, typename T>
OStream& operator<<(OStream& os, const LEDValues<T>& v)
{
  os << "{"
     << "\n\tfront: " << v.front
     << "\n\tleft: " << v.left
     << "\n\trear: " << v.rear
     << "\n\tright: " << v.right
     << "\n}";
  return os;
}


inline unsigned key_from(uint8_t _set, uint8_t _cmd)
{
  return _cmd + 256 * _set;
}

inline float angular_speed_from_rpm(int rpm)
{
  return 2 * M_PI * rpm / 60.0;
}

inline int rpm_from_angular_speed(int speed)
{
  return int(round(speed * 60.0 / (2 * M_PI)));
}

inline float deg2rad(float value)
{
  return M_PI * value / 180.0;
}

inline float rad2deg(float value)
{
  return 180.0 * value * M_1_PI;
}

template <typename T>
T read(const uint8_t * buffer)
{
  T value;
  uint8_t * bytes = (uint8_t *)&value;
  memcpy(bytes, buffer, sizeof(T));
  return value;
}

template <typename T>
void write(std::vector<uint8_t> & buffer, short index, T value)
{
  uint8_t * bytes = (uint8_t *)&value;
  for (size_t i = 0; i < sizeof(T); i++) {
    buffer[index + i] = bytes[i];
  }
}

#endif /* end of include guard: UTILS_HPP */
