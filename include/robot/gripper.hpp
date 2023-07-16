#ifndef INCLUDE_ROBOT_GRIPPER_HPP_
#define INCLUDE_ROBOT_GRIPPER_HPP_

#include "../utils.hpp"

struct Gripper {
  friend class Robot;
  enum Status { pause = 0, open = 1, close = 2 };

  ControllableValue<Status> state;
  ControllableValue<float> power;

  Gripper()
      : state(Status::pause)
      , power(0.5) {}
  /**
   * Control the gripper
   * @param state the desired state: open, close or pause the gripper
   * @param power The fraction of power to apply
   */
  void set_target(Status _state, float _power) {
    state.desired = _state;
    power.desired = _power;
  }

  Status get_status() const { return state.current; }
};


#if FMT_VERSION >= 90000 
template <> struct fmt::formatter<Gripper::Status>: formatter<std::string_view> {
  auto format(Gripper::Status value, format_context& ctx) const {
  std::string_view name = "";
  switch (value) {
    case Gripper::Status::pause: name = "pause"; break;
    case Gripper::Status::open: name = "open"; break;
    case Gripper::Status::close: name = "close"; break;
  }
  return formatter<string_view>::format(name, ctx);    
  }
};
#endif


#endif  //  INCLUDE_ROBOT_GRIPPER_HPP_ */
