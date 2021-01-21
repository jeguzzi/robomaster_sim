#ifndef RT_DUMMY_ROBOT_HPP
#define RT_DUMMY_ROBOT_HPP

#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include  "dummy_robot.hpp"

class RealTimeDummyRobot : public DummyRobot
{
public:
  RealTimeDummyRobot(boost::asio::io_context * io_context, float control_step=0.05);
  ~RealTimeDummyRobot();
private:
  boost::asio::io_context * io_context;
  std::shared_ptr<boost::asio::deadline_timer> timer;
  boost::posix_time::time_duration control_step;
  void cb();
};

#endif /* end of include guard: RT_DUMMY_ROBOT_HPP */
