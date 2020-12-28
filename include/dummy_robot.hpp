#ifndef DUMMY_ROBOT_HPP
#define DUMMY_ROBOT_HPP

#include  "robot.hpp"

class DummyRobot : public Robot
{
public:
  DummyRobot();
  void set_wheel_speeds(float front_right, float front_left, float rear_left, float rear_right);
  void set_leds(Color color, LedMask mask, LedEffect effect, float period_on, float period_off, bool loop);
  void set_velocity(float x, float y, float theta);
  void set_enable_sdk(bool value);
  void control_gripper(GripperStatus state, float power);
};

#endif /* end of include guard: DUMMY_ROBOT_HPP */
