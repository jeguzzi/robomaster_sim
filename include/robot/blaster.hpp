#ifndef INCLUDE_ROBOT_BLASTER_HPP_
#define INCLUDE_ROBOT_BLASTER_HPP_

#include "../utils.hpp"

struct BlasterLED {
  BlasterLED()
      : intensity(0) {}

  ControllableValue<float> intensity;
};

#endif  // INCLUDE_ROBOT_BLASTER_HPP_ */
