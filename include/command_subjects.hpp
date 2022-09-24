#ifndef INCLUDE_COMMAND_SUBJECTS_HPP_
#define INCLUDE_COMMAND_SUBJECTS_HPP_

#include <algorithm>
#include <string>
#include <vector>

#include "spdlog/spdlog.h"

#include "robot/robot.hpp"
#include "subject.hpp"
#include "utils.hpp"

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
// * DDS_SERVO: 0x000200095f0059e7,
// * DDS_ARM: 0x0002000926abd64d,
// * DDS_GRIPPER: 0x00020009124d156a,
// * DDS_GIMBAL_POS: 0x00020009f79b3c97,
// DDS_STICK: 0x0002000955e9a0fa,
// DDS_MOVE_MODE: 0x00020009784c7bfd,
// * DDS_TOF: 0x0002000986e4c05a,
// * DDS_PINBOARD: 0x00020009eebb9ffc,

// NOTE(jerome): Currently the firmware set the angular component to 0
Twist2D from_robot(Twist2D twist) { return {twist.x, -twist.y, rad2deg(0.0)}; }

// NOTE(jerome): Currently the firmware set the angular component to 0
Pose2D from_robot(Pose2D pose) { return {pose.x, -pose.y, rad2deg(0.0)}; }

Attitude from_robot(Attitude attitude) {
  return {.yaw = rad2deg(normalize(-attitude.yaw)),
          .pitch = rad2deg(normalize(-attitude.pitch)),
          .roll = rad2deg(normalize(attitude.roll))};
}

struct VelocitySubject : SubjectWithUID<0x0002000949a4009c> {
  std::string name() { return "Velocity"; }

  // {vgx, vgy, vgz}: The velocity in the world coordinate system [initialized] at the time of
  // power-on
  float vgx;
  float vgy;
  float vgz;
  // {vbx, vby, vbz}: The velocity in the vehicle body coordinate system at the current moment
  float vbx;
  float vby;
  float vbz;

  std::vector<uint8_t> encode() {
    std::vector<uint8_t> buffer(4 * 6, 0);
    write<float>(buffer, 4 * 0, vgx);
    write<float>(buffer, 4 * 1, vgy);
    write<float>(buffer, 4 * 2, vgz);
    write<float>(buffer, 4 * 3, vbx);
    write<float>(buffer, 4 * 4, vby);
    write<float>(buffer, 4 * 5, vbz);
    return buffer;
  }

  void update(Robot *robot) {
    Twist2D twist_odom = from_robot(robot->chassis.get_twist(Frame::odom));
    Twist2D twist_body = from_robot(robot->chassis.get_twist(Frame::body));
    vgx = twist_odom.x;
    vgy = twist_odom.y;
    vgz = twist_odom.theta;
    vbx = twist_body.x;
    vby = twist_body.y;
    vbz = twist_body.theta;
  }
};

struct PositionSubject final : SubjectWithUID<0x00020009eeb7cece> {
  std::string name() { return "Position"; }

  // position3D in [m, m, m]
  float position_x;
  float position_y;
  float position_z;

  std::vector<uint8_t> encode() {
    std::vector<uint8_t> buffer(4 * 3, 0);
    write<float>(buffer, 4 * 0, position_x);
    write<float>(buffer, 4 * 1, position_y);
    write<float>(buffer, 4 * 2, position_z);
    return buffer;
  }

  void update(Robot *robot) {
    Pose2D pose_odom = from_robot(robot->chassis.get_pose());
    position_x = pose_odom.x;
    position_y = pose_odom.y;
    position_z = pose_odom.theta;
  }
};

struct AttiInfoSubject : SubjectWithUID<0x000200096b986306> {
  std::string name() { return "AttiInfo"; }
  // degrees
  float yaw;
  float pitch;
  float roll;

  std::vector<uint8_t> encode() {
    std::vector<uint8_t> buffer(4 * 3, 0);
    write<float>(buffer, 4 * 0, yaw);
    write<float>(buffer, 4 * 1, pitch);
    write<float>(buffer, 4 * 2, roll);
    return buffer;
  }

  void update(Robot *robot) {
    Attitude attitude_odom = from_robot(robot->chassis.get_attitude());
    yaw = attitude_odom.yaw;
    pitch = attitude_odom.pitch;
    roll = attitude_odom.roll;
  }
};

struct ChassisModeSubject : SubjectWithUID<0x000200094fcb1146> {
  std::string name() { return "ChassisMode"; }
  // ? not exposed/documented in the Python client library
  uint8_t mis_cur_type;
  // mode; DONE(jerome): is this the same mode as Robot::Mode?
  // => NO: 8 when stopped, 5 when moving (with an action), ...
  uint8_t sdk_cur_type;

  std::vector<uint8_t> encode() { return {mis_cur_type, sdk_cur_type}; }

  void update(Robot *robot) {
    sdk_cur_type = robot->get_mode();
    mis_cur_type = 0;
  }
};

struct EscSubject : SubjectWithUID<0x00020009c14cb7c5> {
  std::string name() { return "Esc"; }
  constexpr static const int max_speed = 8191;
  constexpr static const int min_speed = -8192;
  constexpr static const int max_angle = 32767;

  static int esc_speed(float speed) {
    return std::clamp<int>(rpm_from_angular_speed(speed), EscSubject::min_speed,
                           EscSubject::max_speed);
  }

  static int esc_angle(float angle) {
    angle = angle / (2 * M_PI);
    angle = fmod(angle, 1.0f);
    if (angle < 0)
      angle += 1.0f;
    return static_cast<int>(round(angle * EscSubject::max_angle));
  }

  // [front right, front left, rear left, rear right]

  // rpm in [-8192, 8191]
  int16_t speed[4];
  // angles in [0, 32767] ~ [0, 2\pi]
  int16_t angle[4];
  // 600 units ~ 1 s
  uint32_t timestamp[4];
  // ?
  uint8_t state[4];

  std::vector<uint8_t> encode() {
    std::vector<uint8_t> buffer(4 * (2 + 2 + 4 + 1), 0);
    size_t j = 0;
    for (size_t i = 0; i < 4; i++, j += 2) {
      write<int16_t>(buffer, j, speed[i]);
    }
    for (size_t i = 0; i < 4; i++, j += 2) {
      write<int16_t>(buffer, j, angle[i]);
    }
    for (size_t i = 0; i < 4; i++, j += 4) {
      write<uint32_t>(buffer, j, timestamp[i]);
    }
    for (size_t i = 0; i < 4; i++, j++) {
      buffer[j] = state[i];
    }
    return buffer;
  }

  void update(Robot *robot) {
    // spdlog::warn("EscSubject not implemented");
    WheelSpeeds speeds = robot->chassis.get_wheel_speeds();
    WheelValues<float> angles = robot->chassis.get_wheel_angles();
    float time_ = robot->get_time();

    speed[0] = EscSubject::esc_speed(speeds.front_right);
    speed[1] = -EscSubject::esc_speed(speeds.front_left);
    speed[2] = -EscSubject::esc_speed(speeds.rear_left);
    speed[3] = EscSubject::esc_speed(speeds.rear_right);

    angle[0] = EscSubject::esc_angle(angles.front_right);
    angle[1] = -EscSubject::esc_angle(angles.front_left);
    angle[2] = -EscSubject::esc_angle(angles.rear_left);
    angle[3] = EscSubject::esc_angle(angles.rear_right);

    for (size_t i = 0; i < 4; i++) {
      timestamp[i] = (int32_t)(600 * time_);
      state[i] = 0;
    }
  }
};

struct ImuSubject : SubjectWithUID<0x00020009a7985b8d> {
  std::string name() { return "Imu"; }
  // [m/s^2]
  static constexpr float G = 9.81f;

  static float acc(float value) { return value / ImuSubject::G; }

  float acc_x, acc_y, acc_z;
  // angular velocity [deg/s]
  float gyro_x, gyro_y, gyro_z;

  std::vector<uint8_t> encode() {
    std::vector<uint8_t> buffer(4 * 6, 0);
    write<float>(buffer, 4 * 0, acc_x);
    write<float>(buffer, 4 * 1, acc_y);
    write<float>(buffer, 4 * 2, acc_z);
    write<float>(buffer, 4 * 3, gyro_x);
    write<float>(buffer, 4 * 4, gyro_y);
    write<float>(buffer, 4 * 5, gyro_z);
    return buffer;
  }

  void update(Robot *robot) {
    IMU imu_body = robot->chassis.get_imu();
    acc_x = acc(imu_body.acceleration.x);
    acc_y = -acc(imu_body.acceleration.y);
    acc_z = -acc(imu_body.acceleration.z);
    // CHANGED: now in radians
    gyro_x = imu_body.angular_velocity.x;
    gyro_y = -imu_body.angular_velocity.y;
    gyro_z = -imu_body.angular_velocity.z;
  }
};

struct SaStatusSubject : SubjectWithUID<0x000200094a2c6d55> {
  static constexpr float MAX_SPEED_STATIC = 10.0f;
  static constexpr float MAX_PITCH_FLAT = 0.1f;
  static constexpr float MAX_ROLL_FLAT = 0.1f;
  static constexpr float MIN_ROLL_OVER = 1.6f;

  std::string name() { return "SaStatus"; }
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

  std::vector<uint8_t> encode() {
    uint8_t byte1 = (static_flag << 0) | (up_hill << 1) | (down_hill << 2) | (on_slope << 3) |
                    (is_pick_up << 4) | (slip_flag << 5) | (impact_x << 6) | (impact_y << 7);
    uint8_t byte2 = (impact_z << 0) | (roll_over << 1) | (hill_static << 2);
    return {byte1, byte2};
  }

  void update(Robot *robot) {
    // TODO(Jerome): Tentative. Some flags are still not clear
    auto speeds = robot->chassis.get_wheel_speeds();
    static_flag = true;
    for (size_t i = 0; i < 4; i++) {
      if (std::abs(speeds[i]) > MAX_SPEED_STATIC) {
        static_flag = false;
        break;
      }
    }
    auto attitude = robot->chassis.get_attitude();
    up_hill = (attitude.pitch < -MAX_PITCH_FLAT);
    down_hill = (attitude.pitch > MAX_PITCH_FLAT);
    on_slope = (std::abs(attitude.roll) > MAX_ROLL_FLAT);
    roll_over = (std::abs(normalize(attitude.roll)) > MIN_ROLL_OVER);
    hill_static = up_hill & static_flag;
  }
};

struct SbusSubject : SubjectWithUID<0x0002000988223568> {
  std::string name() { return "Sbus"; }
  uint8_t connect_status;
  int16_t subs_channel[16];

  std::vector<uint8_t> encode() {
    uint8_t *b = reinterpret_cast<uint8_t *>(&subs_channel);
    std::vector<uint8_t> buffer(b, b + 32);
    buffer.insert(buffer.begin(), connect_status);
    return buffer;
  }

  void update(Robot *robot) {
    // spdlog::warn("SbusSubject not implemented");
    connect_status = false;
  }
};

struct BatterySubject : SubjectWithUID<0x000200096862229f> {
  std::string name() { return "Battery"; }

  uint16_t adc_value;
  int16_t temperature;
  int32_t current;
  uint8_t percent;

  std::vector<uint8_t> encode() {
    std::vector<uint8_t> buffer(10, 0);
    write<uint16_t>(buffer, 0, adc_value);
    write<int16_t>(buffer, 2, temperature);
    write<int32_t>(buffer, 4, current);
    write<uint8_t>(buffer, 8, percent);
    buffer[9] = 1;
    return buffer;
  }

  void update(Robot *robot) {
    // spdlog::warn("BatterySubject update not implemented");
    adc_value = 10987;
    temperature = 321;
    current = -1234;
    percent = 88;
  }
};

struct GripperSubject : SubjectWithUID<0x00020009124d156a> {
  std::string name() { return "Gripper"; }

  uint8_t status;

  std::vector<uint8_t> encode() { return {status}; }

  void update(Robot *robot) { status = robot->gripper.get_status(); }
};

struct ArmSubject : SubjectWithUID<0x0002000926abd64d> {
  std::string name() { return "Arm"; }
  // [mm], TODO(Jerome): check
  uint32_t pos_x, pos_y;

  std::vector<uint8_t> encode() {
    std::vector<uint8_t> buffer(9, 0);
    write<uint32_t>(buffer, 1, pos_x);
    write<uint32_t>(buffer, 5, pos_y);
    return buffer;
  }

  void update(Robot *robot) {
    Vector3 position = robot->arm.get_position();
    pos_x = static_cast<uint32_t>(1000 * position.x);
    pos_y = static_cast<uint32_t>(1000 * position.z);
  }
};

struct ServoSubject : SubjectWithUID<0x000200095f0059e7> {
  std::string name() { return "Servo"; }

  constexpr static int NUMBER_OF_SERVOS = 4;
  // value = - angle * rad2unit + bias
  // reset_value = - reset_angle * rad2unit + bias =>
  // bias = reset_value + reset_angle * rad2unit

  uint8_t valid[NUMBER_OF_SERVOS];
  uint16_t speed[NUMBER_OF_SERVOS];
  uint16_t angle[NUMBER_OF_SERVOS];

  std::vector<uint8_t> encode() {
    std::vector<uint8_t> buffer(NUMBER_OF_SERVOS * 4 + 1, 0);
    buffer[0] = 0;
    for (size_t i = 0; i < NUMBER_OF_SERVOS; i++) {
      buffer[0] += valid[i] << i;
      write<uint16_t>(buffer, 1 + 2 * i, speed[i]);
      write<uint16_t>(buffer, 9 + 2 * i, angle[i]);
    }
    return buffer;
  }

  void update(Robot *robot) {
    for (size_t i = 0; i < 2; i++) {
      angle[i] = servo_angle_value(i, robot->get_servo_angle(i));
      speed[i] = servo_speed_value(i, robot->get_servo_speed(i));
      valid[i] = 1;
    }
    for (size_t i = 2; i < 4; i++) {
      angle[i] = speed[i] = valid[i] = 0;
    }
  }
};

struct TofSubject : SubjectWithUID<0x0002000986e4c05a> {
  std::string name() { return "ToF"; }

  constexpr static size_t NUMBER_OF_TOF = 4;

  static inline uint16_t encode_distance(float value) { return uint16_t(1000 * value); }

  uint8_t cmd_id[NUMBER_OF_TOF];
  uint8_t direct[NUMBER_OF_TOF];
  uint8_t flag[NUMBER_OF_TOF];
  uint16_t distance[NUMBER_OF_TOF];

  std::vector<uint8_t> encode() {
    std::vector<uint8_t> buffer(NUMBER_OF_TOF * 5 + 1, 0);
    for (size_t i = 0; i < NUMBER_OF_TOF; i++) {
      buffer[i * 5] = cmd_id[i];
      buffer[i * 5 + 1] = direct[i];
      buffer[i * 5 + 2] = flag[i];
      buffer[i * 5 + 3] = (distance[i] >> 8) & 0xFF;
      buffer[i * 5 + 4] = distance[i] & 0xFF;
      // spdlog::info("dist {}", distance[i]);
      // write<uint16_t>(buffer, i * 5 + 3, distance[i]);
    }
    return buffer;
  }

  void update(Robot *robot) {
    auto readings = robot->get_tof_readings();
    for (size_t i = 0; i < std::min(NUMBER_OF_TOF, readings.size()); i++) {
      if (readings[i].active) {
        cmd_id[i] = 20;
        direct[i] = 65;
        if (readings[i].distance < 0) {
          flag[i] = 0x0;
          distance[i] = 65534;
        } else {
          flag[i] = 0xFF;
          distance[i] = encode_distance(readings[i].distance);
        }
      } else {
        cmd_id[i] = direct[i] = flag[i] = distance[i] = 0;
      }
    }
  }
};

struct GimbalPosSubject : SubjectWithUID<0x00020009f79b3c97> {
  std::string name() { return "Gimbal"; }

  int16_t yaw_ground_angle;
  int16_t pitch_ground_angle;
  int16_t yaw_angle;
  int16_t pitch_angle;
  uint8_t option_mode;
  uint8_t return_center;

  std::vector<uint8_t> encode() {
    std::vector<uint8_t> buffer(9, 0);
    write<int16_t>(buffer, 0, yaw_ground_angle);
    write<int16_t>(buffer, 2, pitch_ground_angle);
    write<int16_t>(buffer, 4, yaw_angle);
    write<int16_t>(buffer, 6, pitch_angle);
    buffer[8] = (return_center << 2) | option_mode;
    return buffer;
  }

  void update(Robot *robot) {
    auto attitude = robot->gimbal.attitude(Gimbal::Frame::chassis);
    yaw_angle = -round(rad2deg(10 * attitude.yaw));
    pitch_angle = -round(rad2deg(10 * attitude.pitch));
    attitude = robot->gimbal.attitude(Gimbal::Frame::fixed);
    yaw_ground_angle = -round(rad2deg(10 * attitude.yaw));
    pitch_ground_angle = -round(rad2deg(10 * attitude.pitch));
  }
};

struct AdapterSubject : SubjectWithUID<0x00020009eebb9ffc> {
  std::string name() { return "SensorAdapter"; }

  std::vector<uint8_t> encode() {
    std::vector<uint8_t> buffer(36, 0);
    // TODO(Jerome): Pay attention, here 6 sensor [adapters]
    // while `SensorGetData` seems to support 8
    for (size_t i = 0; i < 6; i++) {
      // io port 0
      buffer[6 * i] = 0;
      // io port 1
      buffer[6 * i + 1] = 0;
      // adc port 1
      write<int16_t>(buffer, 6 * i + 2, 548);
      // adc port 2
      write<int16_t>(buffer, 6 * i + 4, 548);
    }
    return buffer;
  }

  void update(Robot *robot) {
    spdlog::warn("AdapterSubject has dummy values");
  }
};



#endif  // INCLUDE_COMMAND_SUBJECTS_HPP_
