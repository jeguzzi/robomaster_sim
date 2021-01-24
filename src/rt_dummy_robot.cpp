#include <spdlog/spdlog.h>

#include "rt_dummy_robot.hpp"

void RealTimeDummyRobot::cb() {
  // spdlog::info("[Dummy] do_step");
  timer->expires_at(timer->expires_at() + control_step);
  do_step(control_step.total_milliseconds() * 0.001);
  timer->async_wait(std::bind(&RealTimeDummyRobot::cb, this));
}

RealTimeDummyRobot::RealTimeDummyRobot(boost::asio::io_context * _io_context, float _control_step)
: DummyRobot(), io_context(_io_context) {
  control_step = boost::posix_time::millisec(long(_control_step * 1000));
  timer = std::make_unique<boost::asio::deadline_timer>(*io_context, control_step);
  timer->async_wait(std::bind(&RealTimeDummyRobot::cb, this));
};


RealTimeDummyRobot::~RealTimeDummyRobot() {
  timer->cancel();
}
