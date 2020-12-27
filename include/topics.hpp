#ifndef TOPIC_H
#define TOPIC_H

#include <boost/asio.hpp>
#include "spdlog/spdlog.h"

#include "server.hpp"
#include "messages.hpp"

class Commands;

struct Topic
{
  uint16_t period_ms;
  Commands * server;
  virtual std::vector<uint8_t> encode() = 0;
  virtual void update() = 0;
  boost::asio::io_context * io_context;
  std::shared_ptr<boost::asio::deadline_timer> timer;
  std::shared_ptr<AddSubMsg::Request> request;

  Topic(boost::asio::io_context * _io_context, Commands * _server, std::shared_ptr<AddSubMsg::Request> _request)
  {
    server = _server;
    io_context = _io_context;
    request = _request;
  }

  ~Topic()
  {
    stop();
  }

  void start();
  void stop();
  void publish();

};

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
    // std::cout << "vgx " << vgx <<  " buffer " << int(buffer[0]) << ", " << int(buffer[1]) << ", " << int(buffer[2]) << ", " << int(buffer[3]) << "\n";
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

// DDS_BATTERY: 0x000200096862229f,
// DDS_GIMBAL_BASE: 0x00020009f5882874,
// DDS_VELOCITY: 0x0002000949a4009c,
// DDS_ESC: 0x00020009c14cb7c5,
// DDS_ATTITUDE: 0x000200096b986306,
// DDS_IMU: 0x00020009a7985b8d,
// DDS_POSITION: 0x00020009eeb7cece,
// DDS_SA_STATUS: 0x000200094a2c6d55,
// DDS_CHASSIS_MODE: 0x000200094fcb1146,
// DDS_SBUS: 0x0002000988223568,
// DDS_SERVO: 0x000200095f0059e7,
// DDS_ARM: 0x0002000926abd64d,
// DDS_GRIPPER: 0x00020009124d156a,
// DDS_GIMBAL_POS: 0x00020009f79b3c97,
// DDS_STICK: 0x0002000955e9a0fa,
// DDS_MOVE_MODE: 0x00020009784c7bfd,
// DDS_TOF: 0x0002000986e4c05a,
// DDS_PINBOARD: 0x00020009eebb9ffc,

#endif /* end of include guard: TOPIC_H */
