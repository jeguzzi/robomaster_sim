#ifndef DUMMY_ROBOT_HPP
#define DUMMY_ROBOT_HPP

#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>

#include  "robot.hpp"

class DummyRobot : public Robot
{
public:
  DummyRobot(boost::asio::io_context * io_context, float control_step=0.05);
  ~DummyRobot();
  void update_led_colors(LEDColors &);
  void update_target_wheel_speeds(WheelSpeeds &);
  WheelSpeeds read_wheel_speeds();
  WheelSpeeds read_wheel_angles();
  IMU read_imu();
  std::vector<unsigned char> read_camera_image();
private:
  boost::asio::io_context * io_context;
  std::shared_ptr<boost::asio::deadline_timer> timer;
  boost::posix_time::time_duration control_step;
  void cb();
  // void control_gripper(GripperStatus state, float power);
};

#endif /* end of include guard: DUMMY_ROBOT_HPP */
