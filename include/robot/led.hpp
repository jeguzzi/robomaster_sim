#ifndef INCLUDE_ROBOT_LED_HPP_
#define INCLUDE_ROBOT_LED_HPP_

#include <algorithm>
#include <map>

#include "../utils.hpp"

using LedMask = uint8_t;

enum Led {
  ARMOR_BOTTOM_BACK = 0x1,
  ARMOR_BOTTOM_FRONT = 0x2,
  ARMOR_BOTTOM_LEFT = 0x4,
  ARMOR_BOTTOM_RIGHT = 0x8,
  ARMOR_TOP_LEFT = 0x10,
  ARMOR_TOP_RIGHT = 0x20,
};

struct Color {
  // [0, 1]
  float r;
  float g;
  float b;

  inline bool operator==(const Color &rhs) const { return r == rhs.r && g == rhs.g && b == rhs.b; }

  inline Color operator*(float f) {
    return {std::clamp(r * f, 0.0f, 1.0f), std::clamp(g * f, 0.0f, 1.0f),
            std::clamp(b * f, 0.0f, 1.0f)};
  }

  inline bool operator!=(const Color &rhs) { return !(*this == rhs); }
};

template <typename OStream> OStream &operator<<(OStream &os, const Color &v) {
  os << "Color <" << int(v.r) << ", " << int(v.g) << ", " << int(v.b) << ">";
  return os;
}

class ActiveLED {
 public:
  enum LedEffect { off = 0, on = 1, breath = 2, flash = 3, scrolling = 4 };

  ActiveLED();

  void update(Color _color, LedEffect _effect, float _period_1, float _period_2, bool _loop);

  void do_step(float time_step);
  bool check() {
    bool v = changed;
    changed = false;
    return v;
  }
  Color color;

 protected:
  bool active;
  Color tcolor;
  LedEffect effect;
  float period_1;
  float period_2;
  float period;
  bool loop;
  float _time;
  unsigned changed;
};

using CompositeLedMask = uint8_t;

class CompositeLED : public ActiveLED {
 public:
  explicit CompositeLED(size_t number = 1);
  void update(Color _color, LedEffect _effect, float _period_1, float _period_2, bool _loop,
              CompositeLedMask mask);
  Color get_color(size_t index) {
    if (effect == LedEffect::on || effect == LedEffect::off) {
      if (colors.count(index)) {
        return colors[index];
      }
      return {};
    }
    return color;
  }
  bool check(size_t index) {
    bool value = changed & (1 << index);
    changed = (changed & ~(1U << index)) | (value << index);
    return value;
  }
  size_t number;

 private:
  std::map<size_t, Color> colors;
};

struct ChassisLED : ChassisLEDValues<ActiveLED> {
  void do_step(float time_step) {
    rear.do_step(time_step);
    front.do_step(time_step);
    left.do_step(time_step);
    right.do_step(time_step);
  }
};

struct GimbalLED : GimbalLEDValues<CompositeLED> {
  GimbalLED() {
    top_left = CompositeLED(8);
    top_right = CompositeLED(8);
  }

  void do_step(float time_step) {
    top_left.do_step(time_step);
    top_right.do_step(time_step);
  }
};

#endif  // INCLUDE_ROBOT_LED_HPP_ */
