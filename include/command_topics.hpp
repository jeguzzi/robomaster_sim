#ifndef COMMAND_TOPIC_H
#define COMMAND_TOPIC_H

#include "topics.hpp"
#include "server.hpp"


struct VelocitySubject final : Topic
{
  using Topic::Topic;
  //name = dds.DDS_VELOCITY
  // uid = dds.SUB_UID_MAP[name]
  float vgx;
  float vgy;
  float vgz;
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

  void update(){
    vgx += 0.01;
    vgy += 0.02;
    vgz += 0.3;
    vbx += -0.01;
    vby += -0.02;
    vbz += -0.3;
  }

};


struct PositionSubject final : Topic
{
  using Topic::Topic;
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

  void update(){
    position_x += 0.1;
    position_y += 0.1;
    position_z += 0.1;
  }
};

struct AttiInfoSubject final : Topic
{
  using Topic::Topic;
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

  void update(){
    yaw += 0.1;
    pitch -= 0.1;
    roll += 0.2;
  }
};


struct ChassisModeSubject final : Topic
{
  using Topic::Topic;
  uint8_t mis_cur_type = 0;
  uint8_t sdk_cur_type = 0;

  std::vector<uint8_t> encode()
  {
    return {mis_cur_type, sdk_cur_type};
  }

  void update(){
    mis_cur_type = 1;
    sdk_cur_type = 2;
  }
};


struct EscSubject final : Topic
{
  using Topic::Topic;
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

  void update(){
    //
  }
};

struct ImuSubject final : Topic
{
  using Topic::Topic;
  float acc_x, acc_y, acc_z;
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

  void update(){
    //
  }
};

struct SaStatusSubject final : Topic
{
  using Topic::Topic;
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

  void update(){
    //
  }
};


struct SbusSubject final : Topic
{
  using Topic::Topic;
  uint8_t connect_status;
  int16_t subs_channel[16];

  std::vector<uint8_t> encode()
  {
    uint8_t * b = (uint8_t *)&subs_channel;
    std::vector<uint8_t> buffer(b, b + 32);
    buffer.insert(buffer.begin(), connect_status);
    return buffer;
  }

  void update(){

  }
};

struct BatterySubject final : Topic
{
  using Topic::Topic;
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

  void update(){

  }
};


struct GripperSubject final : Topic
{

  enum Status: uint8_t {
    open = 1,
    close = 2,
    normal = 0,
  };
  using Topic::Topic;
  Status status;

  std::vector<uint8_t> encode()
  {
    return {status};
  }

  void update(){

  }
};

struct ArmSubject final : Topic
{

  using Topic::Topic;

  uint32_t pos_x, pos_y;

  std::vector<uint8_t> encode()
  {
    std::vector<uint8_t> buffer(9, 0);
    write<uint32_t>(buffer, 1, pos_x);
    write<uint32_t>(buffer, 5, pos_y);
    return buffer;
  }

  void update(){

  }
};


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

#endif /* end of include guard: COMMAND_TOPIC_H */
