#ifndef INCLUDE_ROBOT_ARMOR_HPP_
#define INCLUDE_ROBOT_ARMOR_HPP_

#include <cstdint>
#include <vector>

struct HitEvent {
  uint8_t type;
  uint8_t index;
};

using hit_event_t = std::vector<HitEvent>;

struct RobotIREvent {
  uint8_t skill_id;
  uint8_t role_id;
  uint8_t recv_dev;
  uint8_t recv_ir_pin;
};

using ir_event_t = std::vector<RobotIREvent>;

struct Armor {
  hit_event_t hit_events;
  ir_event_t ir_events;
  hit_event_t get_hit_events() const { return hit_events; }
  ir_event_t get_ir_events() const { return ir_events; }
};

#endif  //  INCLUDE_ROBOT_ARMOR_HPP_ */
