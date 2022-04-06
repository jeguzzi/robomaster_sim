#ifndef INCLUDE_ROBOT_TOF_HPP_
#define INCLUDE_ROBOT_TOF_HPP_

#include <vector>
#include "spdlog/spdlog.h"

#define MAX_NUMBER_OF_TOF_SENSORS 4

struct ToFReading {
  float distance;
  bool active;

  ToFReading() : distance(0.0), active(false) {}
};

using ToFReadings = std::array<ToFReading, MAX_NUMBER_OF_TOF_SENSORS>;

struct ToF {
  ToFReadings readings;
  bool set_enable(size_t index, bool value) {
    if (index < MAX_NUMBER_OF_TOF_SENSORS) {
      readings[index].active = value;
      spdlog::info("{} distance sensor {}", value ? "Enabled" : "Disabled", index);
      return true;
    }
    return false;
  }
};

#endif  //  INCLUDE_ROBOT_TOF_HPP_ */
