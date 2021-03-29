#ifndef INCLUDE_ROBOT_CAMERA_HPP_
#define INCLUDE_ROBOT_CAMERA_HPP_

#include <vector>

#include "../utils.hpp"

using Image = std::vector<uint8_t>;

struct Camera {
  int width;
  int height;
  float fps;
  bool streaming;
  Image image;
  Camera()
      : streaming(false) {}
};

#endif  //  INCLUDE_ROBOT_CAMERA_HPP_ */
