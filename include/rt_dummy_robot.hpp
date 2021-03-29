#ifndef INCLUDE_RT_DUMMY_ROBOT_HPP_
#define INCLUDE_RT_DUMMY_ROBOT_HPP_

#include <memory>

#include "dummy_robot.hpp"
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>

class RealTimeDummyRobot : public DummyRobot {
 public:
  explicit RealTimeDummyRobot(boost::asio::io_context *io_context, float control_step = 0.05,
                              bool _has_arm = false, bool _has_gripper = false,
                              ServoValues<bool> _has_servo = {}, bool _has_gimbal = false,
                              bool _has_camera = false, bool _has_tof = false);
  ~RealTimeDummyRobot();

 private:
  boost::asio::io_context *io_context;
  std::unique_ptr<boost::asio::deadline_timer> timer;
  boost::posix_time::time_duration control_step;
  void cb();
};

#endif  // INCLUDE_RT_DUMMY_ROBOT_HPP_
