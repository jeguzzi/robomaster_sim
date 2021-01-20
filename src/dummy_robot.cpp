#include <spdlog/spdlog.h>

#include "dummy_robot.hpp"

static std::vector<unsigned char> generate_strip_image(unsigned i0, unsigned i1, unsigned width, unsigned height) {
  unsigned size = width * height * 3;
  std::vector<unsigned char> buffer(size, 0);
  if(i0 > i1) i1 += width;
  for (size_t i = i0; i < i1; i++)
    for (size_t j = 0; j < height; j++)
      buffer[(3 * (j * width + i)) % size] = 255;
  return buffer;
}

std::vector<unsigned char> DummyRobot::read_camera_image() {
  static unsigned seq = 0;
  seq = (seq + 1) % camera_width;
  return generate_strip_image(seq, seq + 10, camera_width, camera_height);
}

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
  IMU imu;
  imu.attitude.yaw = get_pose().theta;
  imu.acceleration.z = 9.81;
  return imu;
}

DummyRobot::~DummyRobot()
{
  timer->cancel();
}

bool DummyRobot::set_camera_resolution(unsigned width, unsigned height) {
  camera_width = width;
  camera_height = height;
  return true;
}

void DummyRobot::update_target_servo_angles(ServoValues<float> &angles) {

}

ServoValues<float> DummyRobot::read_servo_angles() {
  return {
    0.5f * (get_servo_angles().right + target_servo_angles.right),
    0.5f * (get_servo_angles().left + target_servo_angles.left)};
}

ServoValues<float> DummyRobot::read_servo_speeds() {
  return {
    0.5f * (get_servo_angles().right + target_servo_angles.right) / last_time_step,
    0.5f * (get_servo_angles().left + target_servo_angles.left) / last_time_step
  };
}



void DummyRobot::update_target_gripper(Robot::GripperStatus state, float power) {

}

Robot::GripperStatus DummyRobot::read_gripper_state() {
  return target_gripper_state;
}
