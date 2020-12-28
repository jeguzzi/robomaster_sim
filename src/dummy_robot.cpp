#include "dummy_robot.hpp"

DummyRobot::DummyRobot()
: Robot() {

};

void DummyRobot::set_wheel_speeds(float front_right, float front_left, float rear_left, float rear_right) {

}

void DummyRobot::set_leds(Color color, Robot::LedMask mask, Robot::LedEffect effect, float period_on, float period_off, bool loop) {

}

void DummyRobot::set_velocity(float x, float y, float theta) {

}

void DummyRobot::set_enable_sdk(bool value) {

}

void DummyRobot::control_gripper(Robot::GripperStatus state, float power) {

}
