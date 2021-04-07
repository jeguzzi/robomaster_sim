#include "spdlog/spdlog.h"

#include "rt_dummy_robot.hpp"

void RealTimeDummyRobot::cb() {
  // spdlog::info("[Dummy] do_step");
  timer->expires_at(timer->expires_at() + control_step);
  do_step(control_step.total_milliseconds() * 0.001);
  timer->async_wait(std::bind(&RealTimeDummyRobot::cb, this));
}

RealTimeDummyRobot::RealTimeDummyRobot(boost::asio::io_context *_io_context, float _control_step,
                                       bool _has_arm, bool _has_gripper,
                                       ServoValues<bool> _has_servo, bool _has_gimbal,
                                       bool _has_camera, bool _has_tof)
    : DummyRobot(_has_arm, _has_gripper, _has_servo, _has_gimbal, _has_camera, _has_tof)
    , io_context(_io_context) {
  spdlog::info("Created real time dummy robot that will run an update step every {} s ",
               _control_step);
  control_step = boost::posix_time::millisec(unsigned(_control_step * 1000));
  timer = std::make_unique<boost::asio::deadline_timer>(*io_context, control_step);
  timer->async_wait(std::bind(&RealTimeDummyRobot::cb, this));
}

RealTimeDummyRobot::~RealTimeDummyRobot() { timer->cancel(); }
