#include "spdlog/spdlog.h"

#include "robot/led.hpp"

static Color breath_led(float _time, Color color, float period_1, float period_2) {
  float f;
  if (_time < period_1) {
    f = std::sin(_time / period_1 * M_PI_2);
  } else {
    f = std::cos((_time - period_1) / period_2 * M_PI_2);
  }
  // spdlog::debug("breath {} {}: {} -> {}", period_1, period_2, _time, f);
  return color * (f * f);
}

static Color flash_led(float _time, Color color, float period_1, float period_2) {
  if (_time < period_1)
    return color;
  return {};
}

static Color scroll_led(float _time, Color color, float period_1, float period_2) {
  if (_time < 0.175f)
    return color;
  if (_time < (0.175f + period_1))
    return {};
  return color;
}

ActiveLED::ActiveLED()
    : color()
    , active(false)
    , _time(0) {}

void ActiveLED::update(Color _color, LedEffect _effect, float _period_1, float _period_2,
                       bool _loop) {
  tcolor = _color;
  period_1 = _period_1;
  period_2 = _period_2;
  loop = _loop;
  effect = _effect;
  period = _period_1 + _period_2;
  if (effect == LedEffect::off)
    color = {};
  if (effect == LedEffect::on)
    color = tcolor;
  if (effect == LedEffect::scrolling)
    period = 0.175 + _period_1 + 7.5 * period_2;
  active = true;
  _time = 0;
  // active = (effect != LedEffect::off && effect != LedEffect::on);
  // if (active)
  //   _time = 0;
  changed = true;
}

void ActiveLED::do_step(float time_step) {
  if (!active) {
    changed = 0;
    return;
  }
  changed = 0xFF;
  if (effect == LedEffect::off || effect == LedEffect::on) {
    active = false;
    return;
  }
  _time = _time + time_step;
  if (!loop && _time > period) {
    active = false;
    // TODO(Jerome): Check which color at the end of an effect
    return;
  }
  _time = fmod(_time, period);
  if (effect == LedEffect::flash) {
    color = flash_led(_time, tcolor, period_1, period_2);
  } else if (effect == LedEffect::breath) {
    color = breath_led(_time, tcolor, period_1, period_2);
  } else if (effect == LedEffect::scrolling) {
    color = scroll_led(_time, tcolor, period_1, period_2);
  }
}

CompositeLED::CompositeLED(size_t number)
    : ActiveLED::ActiveLED()
    , number(number) {}

void CompositeLED::update(Color _color, LedEffect _effect, float _period_1, float _period_2,
                          bool _loop, CompositeLedMask _led_mask) {
  ActiveLED::update(_color, _effect, _period_1, _period_2, _loop);
  if (effect == LedEffect::off || effect == LedEffect::on) {
    for (size_t i = 0; i < number; i++) {
      if (_led_mask & (1 << i)) {
        if (effect == LedEffect::off) {
          colors[i] = {};
        } else {
          colors[i] = color;
        }
      }
    }
  } else {
    colors.clear();
    _led_mask = 0xFF;
  }
  changed = _led_mask;
}
