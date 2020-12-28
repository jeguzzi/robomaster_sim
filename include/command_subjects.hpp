#ifndef COMMAND_SUBJECTS_H
#define COMMAND_SUBJECTS_H

#include "subject.hpp"
#include "utils.hpp"
#include <spdlog/spdlog.h>

// * DDS_BATTERY: 0x000200096862229f,
// DDS_GIMBAL_BASE: 0x00020009f5882874,
// * DDS_VELOCITY: 0x0002000949a4009c,
// * DDS_ESC: 0x00020009c14cb7c5,
// * DDS_ATTITUDE: 0x000200096b986306,
// * DDS_IMU: 0x00020009a7985b8d,
// * DDS_POSITION: 0x00020009eeb7cece,
// * DDS_SA_STATUS: 0x000200094a2c6d55,
// * DDS_CHASSIS_MODE: 0x000200094fcb1146,
// * DDS_SBUS: 0x0002000988223568,
// DDS_SERVO: 0x000200095f0059e7,
// * DDS_ARM: 0x0002000926abd64d,
// * DDS_GRIPPER: 0x00020009124d156a,
// DDS_GIMBAL_POS: 0x00020009f79b3c97,
// DDS_STICK: 0x0002000955e9a0fa,
// DDS_MOVE_MODE: 0x00020009784c7bfd,
// DDS_TOF: 0x0002000986e4c05a,
// DDS_PINBOARD: 0x00020009eebb9ffc,

struct VelocitySubject : SubjectWithUID<0x0002000949a4009c>
{

  // {vgx, vgy, vgz}: The velocity in the world coordinate system [initialized] at the time of power-on
  float vgx;
  float vgy;
  float vgz;
  // {vbx, vby, vbz}: The velocity in the vehicle body coordinate system at the current moment
  float vbx;
  float vby;
  float vbz;

  std::vector<uint8_t> encode()
  {
    std::vector<uint8_t> buffer(4 * 6, 0);
    write<float>(buffer, 4 * 0, vgx);
    write<float>(buffer, 4 * 1, vgy);
    write<float>(buffer, 4 * 2, vgz);
    write<float>(buffer, 4 * 3, vbx);
    write<float>(buffer, 4 * 4, vby);
    write<float>(buffer, 4 * 5, vbz);
    return buffer;
  }

  void update(Robot * robot){
    Twist2D twist_odom = robot->get_twist(Robot::Frame::odom);
    Twist2D twist_body = robot->get_twist(Robot::Frame::body);
    vgx = twist_odom.x;
    vgy = twist_odom.y;
    vgz = twist_odom.theta;
    vbx = twist_body.x;
    vby = twist_body.y;
    vbz = twist_body.theta;
  }

};


struct PositionSubject final : SubjectWithUID<0x00020009eeb7cece>
{

  // pose2D in [m, m, degree]
  float position_x;
  float position_y;
  float position_z;

  std::vector<uint8_t> encode()
  {
    std::vector<uint8_t> buffer(4 * 3, 0);
    write<float>(buffer, 4 * 0, position_x);
    write<float>(buffer, 4 * 1, position_y);
    write<float>(buffer, 4 * 2, position_z);
    return buffer;
  }

  void update(Robot * robot){
    Pose2D pose_odom = robot->get_pose();
    position_x = pose_odom.x;
    position_y = pose_odom.y;
    position_z = rad2deg(pose_odom.theta);
  }
};

struct AttiInfoSubject : SubjectWithUID<0x000200096b986306>
{
  // degrees
  float yaw;
  float pitch;
  float roll;

  std::vector<uint8_t> encode()
  {
    std::vector<uint8_t> buffer(4 * 3, 0);
    write<float>(buffer, 4 * 0, yaw);
    write<float>(buffer, 4 * 1, pitch);
    write<float>(buffer, 4 * 2, roll);
    return buffer;
  }

  void update(Robot * robot){
    Attitude attitude_odom = robot->get_attitude();
    yaw = rad2deg(attitude_odom.yaw);
    pitch = rad2deg(attitude_odom.pitch);
    roll = rad2deg(attitude_odom.roll);
  }
};


struct ChassisModeSubject : SubjectWithUID<0x000200094fcb1146>
{
  // ? not exposed/documented in the Python client library
  uint8_t mis_cur_type;
  // mode; TODO(jerome): is this the same mode as Robot::Mode?
  uint8_t sdk_cur_type;

  std::vector<uint8_t> encode()
  {
    return {mis_cur_type, sdk_cur_type};
  }

  void update(Robot * robot){
    sdk_cur_type = robot->get_mode();
    mis_cur_type = 0;
  }
};


struct EscSubject : SubjectWithUID<0x00020009c14cb7c5>
{

  int16_t speed[4];
  int16_t angle[4];
  uint32_t timestamp[4];
  uint8_t state[4];

  std::vector<uint8_t> encode()
  {
    std::vector<uint8_t> buffer(4 * (2 + 2 + 4 + 1), 0);
    short j = 0;
    for (size_t i = 0; i < 4; i++, j+=2) {
        write<int16_t>(buffer, j, speed[i]);
    }
    for (size_t i = 0; i < 4; i++, j+=2) {
        write<int16_t>(buffer, j, angle[i]);
    }
      for (size_t i = 0; i < 4; i++, j+=4) {
        write<uint32_t>(buffer, j, timestamp[i]);
    }
    for (size_t i = 0; i < 4; i++, j++) {
        buffer[j]=state[i];
    }
    return buffer;
  }

  void update(Robot * robot){
    spdlog::warn("EscSubject not implemented");
  }
};

struct ImuSubject : SubjectWithUID<0x00020009a7985b8d>
{
  // [m/s^2]
  float acc_x, acc_y, acc_z;
  // angular velocity [deg/s]
  float gyro_x, gyro_y, gyro_z;

  std::vector<uint8_t> encode()
  {
    std::vector<uint8_t> buffer(4 * 6, 0);
    write<float>(buffer, 4 * 0, acc_x);
    write<float>(buffer, 4 * 1, acc_y);
    write<float>(buffer, 4 * 2, acc_z);
    write<float>(buffer, 4 * 3, gyro_x);
    write<float>(buffer, 4 * 4, gyro_y);
    write<float>(buffer, 4 * 5, gyro_z);
    return buffer;
  }

  void update(Robot * robot){
    IMU imu_body = robot->get_imu();
    acc_x = imu_body.acceleration.x;
    acc_y = imu_body.acceleration.y;
    acc_z = imu_body.acceleration.z;
    gyro_x = imu_body.angular_velocity.x;
    gyro_y = imu_body.angular_velocity.y;
    gyro_z = imu_body.angular_velocity.z;
  }
};

struct SaStatusSubject : SubjectWithUID<0x000200094a2c6d55>
{
  // Status standard bit [?]
  bool static_flag;
  bool up_hill;
  bool down_hill;
  bool on_slope;
  bool is_pick_up;
  bool slip_flag;
  bool impact_x;
  bool impact_y;
  bool impact_z;
  bool roll_over;
  bool hill_static;

  std::vector<uint8_t> encode()
  {

    uint8_t byte1 = (static_flag << 0) | (up_hill << 1)   | (down_hill << 2) | (on_slope << 3) |
                    (is_pick_up << 4)  | (slip_flag << 5) | (impact_x << 6)  | (impact_y << 7);
    uint8_t byte2 = (impact_z << 0) | (roll_over << 1)   | (hill_static << 2);
    return {byte1, byte2};
  }

  void update(Robot * robot){
    spdlog::warn("SaStatusSubject update not implemented");
  }
};


struct SbusSubject : SubjectWithUID<0x0002000988223568>
{
  uint8_t connect_status;
  int16_t subs_channel[16];

  std::vector<uint8_t> encode()
  {
    uint8_t * b = (uint8_t *)&subs_channel;
    std::vector<uint8_t> buffer(b, b + 32);
    buffer.insert(buffer.begin(), connect_status);
    return buffer;
  }

  void update(Robot * robot){
    spdlog::warn("SbusSubject not implemented");
  }
};

struct BatterySubject : SubjectWithUID<0x000200096862229f>
{
  uint16_t adc_value;
  int16_t temperature;
  int32_t current;
  uint8_t percent;

  std::vector<uint8_t> encode()
  {
    std::vector<uint8_t> buffer(10, 0);
    write<uint16_t>(buffer, 0, adc_value);
    write<int16_t>(buffer, 2, temperature);
    write<int32_t>(buffer, 4, current);
    write<uint8_t>(buffer, 8, percent);
    buffer[9] = 1;
    return buffer;
  }

  void update(Robot * robot){
    spdlog::warn("BatterySubject update not implemented");
  }
};


struct GripperSubject : SubjectWithUID<0x00020009124d156a>
{
  uint8_t status;

  std::vector<uint8_t> encode()
  {
    return {status};
  }

  void update(Robot * robot){
    status = robot->get_gripper_status();
  }
};

struct ArmSubject : SubjectWithUID<0x0002000926abd64d>
{
  // [mm], TODO(jerome): check
  uint32_t pos_x, pos_y;

  std::vector<uint8_t> encode()
  {
    std::vector<uint8_t> buffer(9, 0);
    write<uint32_t>(buffer, 1, pos_x);
    write<uint32_t>(buffer, 5, pos_y);
    return buffer;
  }

  void update(Robot * robot){
    Vector3 position = robot->get_arm_position();
    pos_x = static_cast<uint32_t>(position.x);
    pos_y = static_cast<uint32_t>(position.z);
  }
};

#endif /* end of include guard: COMMAND_SUBJECTS_H */
