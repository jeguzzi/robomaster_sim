#ifndef INCLUDE_ROBOT_VISION_HPP_
#define INCLUDE_ROBOT_VISION_HPP_

#include <map>
#include <vector>

#include "utils.hpp"

struct DetectedObjects {
  enum Type : uint8_t { PERSON = 1, GESTURE = 2, LINE = 4, MARKER = 5, ROBOT = 7 };

  struct Person {
    constexpr static Type type{PERSON};
    BoundingBox bounding_box;
    explicit Person(BoundingBox bounding_box)
        : bounding_box(bounding_box) {}
  };

  struct Gesture {
    constexpr static Type type{GESTURE};
    BoundingBox bounding_box;
    uint32_t id;
    Gesture(BoundingBox bounding_box, uint32_t id)
        : bounding_box(bounding_box)
        , id(id) {}
  };

  struct Line {
    constexpr static Type type{LINE};
    float x;
    float y;
    float curvature;
    float angle;
    uint32_t info;
    Line(float x, float y, float curvature, float angle)
        : x(x)
        , y(y)
        , curvature(curvature)
        , angle(angle) {}
  };

  struct Marker {
    constexpr static Type type{MARKER};
    BoundingBox bounding_box;
    uint16_t id;
    uint16_t distance;
    // TODO(Jerome): I don't know the distance encoding
    Marker(BoundingBox bounding_box, uint16_t id, float distance)
        : bounding_box(bounding_box)
        , id(id)
        , distance(uint16_t(distance * 1000)) {}
  };

  struct Robot {
    constexpr static Type type{ROBOT};
    BoundingBox bounding_box;
    explicit Robot(BoundingBox bounding_box)
        : bounding_box(bounding_box) {}
  };

  std::vector<Person> people;
  std::vector<Gesture> gestures;
  std::vector<Line> lines;
  std::vector<Marker> markers;
  std::vector<Robot> robots;

  template <typename T> const std::vector<T> &get() const;

  template <> const std::vector<Person> &get() const { return people; }

  template <> const std::vector<Gesture> &get() const { return gestures; }

  template <> const std::vector<Line> &get() const { return lines; }

  template <> const std::vector<Marker> &get() const { return markers; }

  template <> const std::vector<Robot> &get() const { return robots; }
};

struct Vision {
  uint8_t enabled;
  enum Color { RED = 1, GREEN = 2, BLUE = 3 };
  std::map<DetectedObjects::Type, Color> color;
  template <typename T> bool is_enabled() const { return (1 << T::type) & enabled; }
  DetectedObjects detected_objects;

  // ignored for now:
  // color {1: red, 2: green, 3: blue}
  // type {1: line, 2: marker}
  // TODO(Jerome): attention that this enum is different that the one used in vision type
  void set_color(uint8_t type, uint8_t value) {
    Color c = static_cast<Color>(value);
    if (type == 1)
      color[DetectedObjects::Type::LINE] = c;
    else if (type == 2)
      color[DetectedObjects::Type::MARKER] = c;
  }

  void set_enable(uint8_t value) { enabled = value; }
  uint8_t get_enable() { return enabled; }
  const DetectedObjects &get_detected_objects() { return detected_objects; }
};

#endif  // INCLUDE_ROBOT_VISION_HPP_ */
