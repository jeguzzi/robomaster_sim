#include "coppeliasim_robot.hpp"
#include <spdlog/spdlog.h>

void CoppeliaSimRobot::update_target_wheel_speeds(WheelSpeeds & speeds) {
  for (size_t i = 0; i < 4; i++) {
    spdlog::debug("Set wheel joint {} speed to {}", i, speeds[i]);
    simSetJointTargetVelocity(wheel_joint_handles[i], speeds[i]);
  }
}

void CoppeliaSimRobot::update_led_colors(LEDColors & colors)
{
  for (size_t i = 0; i < 4; i++) {
    simFloat color[3] = {colors[i].r, colors[i].g, colors[i].b};
    spdlog::debug("Set led {} color to {} {} {}", i, color[0], color[1], color[2]);
    simSetShapeColor(led_handles[i], nullptr, sim_colorcomponent_emission, color);
  }
}

WheelSpeeds CoppeliaSimRobot::read_wheel_speeds() {
  WheelSpeeds value;
  for (size_t i = 0; i < 4; i++) {
    simGetObjectFloatParameter(wheel_joint_handles[i], sim_jointfloatparam_velocity, &value[i]);
  }
  return value;
}

WheelValues<float> CoppeliaSimRobot::read_wheel_angles() {
  WheelValues<float> value;
  for (size_t i = 0; i < 4; i++) {
    simFloat rvalue;
    simGetJointPosition(wheel_joint_handles[i], &rvalue);
    value[i] = rvalue;
  }
  return value;
}

// DONE(jerome): add force sensor?, simulate gyro/magnetometer
// or maybe just get velocity and compute acceleration in the base class
IMU CoppeliaSimRobot::read_imu() {
  return imu;
}


void CoppeliaSimRobot::has_read_accelerometer(float x, float y, float z) {
  imu.acceleration = {x, y, z};
}

void CoppeliaSimRobot::has_read_gyro(float x, float y, float z) {
  imu.angular_velocity = {x, y, z};
}

void CoppeliaSimRobot::update_orientation(float alpha, float beta, float gamma) {
  imu.attitude.roll = alpha;
  imu.attitude.pitch = beta;
}
