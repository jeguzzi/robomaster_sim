#ifndef INCLUDE_ROBOT_TOF_HPP_
#define INCLUDE_ROBOT_TOF_HPP_

#include <vector>

#define MAX_NUMBER_OF_TOF_SENSORS 4

struct ToFReading {
  float distance;
  bool active;

  ToFReading() : distance(0.0), active(false) {}
};

using ToFReadings = std::array<ToFReading, MAX_NUMBER_OF_TOF_SENSORS>;

struct ToF {
  ToFReadings readings;
  void set_enable(size_t index, bool value) {
    if (index < MAX_NUMBER_OF_TOF_SENSORS) {
      readings[index].active = value;
    }
  }
};

#endif  //  INCLUDE_ROBOT_TOF_HPP_ */
