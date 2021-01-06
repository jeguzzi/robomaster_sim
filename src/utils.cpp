#include "utils.hpp"

float normalize(float value) {
  value = fmod(value + M_PI, 2 * M_PI);
  if (value < 0) value += 2 * M_PI;
  return value - M_PI;
}
