#include <spdlog/spdlog.h>

#include "dummy_robot.hpp"


void DummyRobot::cb() {
  // spdlog::info("[Dummy] do_step");
  timer->expires_at(timer->expires_at() + control_step);
  do_step(control_step.total_milliseconds() * 0.001);
  timer->async_wait(std::bind(&DummyRobot::cb, this));
}

DummyRobot::DummyRobot(boost::asio::io_context * _io_context, float _control_step)
: Robot(), io_context(_io_context) {
  control_step = boost::posix_time::millisec(long(_control_step * 1000));
  timer = std::make_shared<boost::asio::deadline_timer>(*io_context, control_step);
  timer->async_wait(std::bind(&DummyRobot::cb, this));
};

void DummyRobot::update_target_wheel_speeds(WheelSpeeds & speeds) {

}

WheelSpeeds DummyRobot::read_wheel_speeds()
{
  return target_wheel_speed;
}

WheelValues<float> DummyRobot::read_wheel_angles()
{
  return {};
}

void DummyRobot::update_led_colors(LEDColors &colors) {
}

IMU DummyRobot::read_imu() {
  return {};
}

DummyRobot::~DummyRobot()
{
  timer->cancel();
}
