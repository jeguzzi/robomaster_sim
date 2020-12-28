#ifndef UTILS_HPP
#define UTILS_HPP

#define _USE_MATH_DEFINES
#include <cmath>
#include <string>
#include <vector>
#include <cstdint>

inline unsigned key_from(uint8_t _set, uint8_t _cmd)
{
  return _cmd + 256 * _set;
}

inline float angular_speed(int rpm)
{
  return 2 * M_PI * rpm;
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
