#ifndef INCLUDE_ROBOT_TOF_HPP_
#define INCLUDE_ROBOT_TOF_HPP_

#include <vector>

struct ToFReading {
  float distance;
  bool active;
};

struct ToF {
  std::vector<ToFReading> readings;
};

#endif  //  INCLUDE_ROBOT_TOF_HPP_ */
