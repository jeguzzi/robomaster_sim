#include "robot/vision.hpp"

template <> const std::vector<DetectedObjects::Person> &DetectedObjects::get() const {
  return people;
}

template <> const std::vector<DetectedObjects::Gesture> &DetectedObjects::get() const {
  return gestures;
}

template <> const std::vector<DetectedObjects::Line> &DetectedObjects::get() const { return lines; }

template <> const std::vector<DetectedObjects::Marker> &DetectedObjects::get() const {
  return markers;
}

template <> const std::vector<DetectedObjects::Robot> &DetectedObjects::get() const {
  return robots;
}
