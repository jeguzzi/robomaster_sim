#ifndef INCLUDE_COMMAND_MESSAGES_HPP_
#define INCLUDE_COMMAND_MESSAGES_HPP_

#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <boost/asio.hpp>

#include "spdlog/fmt/bin_to_hex.h"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"

#include "action.hpp"
#include "protocol.hpp"
#include "utils.hpp"

uint8_t accept_code(Action::State state) {
  switch (state) {
  case Action::State::running:
  case Action::State::started:
    return 0;
  case Action::State::succeed:
    return 2;
  case Action::failed:
  case Action::undefined:
  case Action::rejected:
    return 1;
  }
}

struct GetVersion : Proto<0x0, 0x1> {
  struct Request : RequestT {
    Request(uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri,
            const uint8_t *buffer)
        : RequestT(_sender, _receiver, _seq_id, _attri) {}

    template <typename OStream> friend OStream &operator<<(OStream &os, const Request &r) {
      os << "GetVersion::Request { }";
      return os;
    }
  };

  struct Response : ResponseT {
    uint8_t aa;
    uint8_t bb;
    uint8_t cc;
    uint8_t dd;
    std::vector<uint8_t> encode() {
      std::vector<uint8_t> buffer(30, 0);
      buffer[0] = aa;
      buffer[1] = bb;
      buffer[2] = cc;
      buffer[3] = dd;
      return buffer;
    }
    using ResponseT::ResponseT;
  };

  static bool answer(const Request &request, Response &response, Robot *robot) {
    response.aa = 0;
    response.bb = 1;
    response.cc = 2;
    response.dd = 3;
    return true;
  }
};

struct GetProductVersion : Proto<0x0, 0x4f> {
  struct Request : RequestT {
    uint8_t file_type;

    Request(uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri,
            const uint8_t *buffer)
        : RequestT(_sender, _receiver, _seq_id, _attri) {
      file_type = buffer[0];
    }

    template <typename OStream> friend OStream &operator<<(OStream &os, const Request &r) {
      os << "GetProductVersion::Request {"
         << " file_type=" << int(r.file_type) << "}";
      return os;
    }
  };

  struct Response : ResponseT {
    uint8_t aa;
    uint8_t bb;
    uint16_t cc;
    std::vector<uint8_t> encode() {
      std::vector<uint8_t> buffer(13, 0);
      buffer[9] = aa;
      buffer[10] = bb;
      write<uint16_t>(buffer, 11, cc);
      return buffer;
    }
    using ResponseT::ResponseT;
  };

  static bool answer(const Request &request, Response &response, Robot *robot) {
    response.aa = 0;
    response.bb = 1;
    response.cc = 2;
    return true;
  }
};

struct GetSn : Proto<0x0, 0x51> {
  struct Request : RequestT {
    uint8_t type;

    Request(uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri,
            const uint8_t *buffer)
        : RequestT(_sender, _receiver, _seq_id, _attri) {
      type = buffer[0];
    }

    template <typename OStream> friend OStream &operator<<(OStream &os, const Request &r) {
      os << "GetSn::Request {"
         << " type=" << int(r.type) << "}";
      return os;
    }
  };

  // TODO(jerome): not sure about the string serialization (should be utf-8)
  struct Response : ResponseT {
    std::string serial_number;

    std::vector<uint8_t> encode() {
      std::vector<uint8_t> buffer(2 + serial_number.size(), 0);
      write<uint16_t>(buffer, 0, serial_number.size());
      for (size_t i = 0; i < serial_number.size(); i++) {
        buffer[2 + i] = (uint8_t)serial_number.data()[i];
      }
      return buffer;
    }
    using ResponseT::ResponseT;
  };

  static bool answer(const Request &request, Response &response, Robot *robot) {
    response.serial_number = "serial_1234";
    return true;
  }
};

/* NOT USED
struct ChassisWheelSpeed : Proto<0x3f, 0x26>
{

  struct Request : RequestT {

    int8_t w1_speed;
    int8_t w2_speed;
    int8_t w3_speed;
    int8_t w4_speed;

    Request (uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri, const uint8_t *
buffer) : RequestT(_sender, _receiver, _seq_id, _attri){ w1_speed = buffer[0]; w2_speed = buffer[1];
        w3_speed = buffer[2];
        w4_speed = buffer[3];
      };

    template<typename OStream>
    friend OStream& operator<<(OStream& os, const Request& r)
    {
      os << "ChassisWheelSpeed::Request {"
         << " w1_speed=" << int(r.w1_speed)
         << " w2_speed=" << int(r.w2_speed)
         << " w3_speed=" << int(r.w3_speed)
         << " w4_speed=" << int(r.w4_speed)
         << "}";
      return os;
    }
  };

  static bool answer(const Request &request, Response &response, Robot  * robot)
  {
    float speed[4] = {angular_speed_from_rpm(request.w1_speed),
angular_speed_from_rpm(request.w2_speed), angular_speed_from_rpm(request.w3_speed),
angular_speed_from_rpm(request.w4_speed)}; robot->set_wheel_speed(speed); return true;
  }

};
*/

struct SetWheelSpeed : Proto<0x3f, 0x20> {
  struct Request : RequestT {
    // int:[-1000,1000] right front [rpm], front robot direction -> positiove speed
    int16_t w1_speed;
    // int:[-1000,1000] left front [rpm], front robot direction -> negative speed
    int16_t w2_speed;
    // int:[-1000,1000] left rear [rpm], front robot direction -> negative speed
    int16_t w3_speed;
    // int:[-1000,1000] right rear [rpm], front robot direction -> positiove speed
    int16_t w4_speed;

    Request(uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri,
            const uint8_t *buffer)
        : RequestT(_sender, _receiver, _seq_id, _attri) {
      w1_speed = read<int16_t>(buffer);
      w2_speed = -read<int16_t>(buffer + 2);
      w3_speed = -read<int16_t>(buffer + 4);
      w4_speed = read<int16_t>(buffer + 6);
    }

    template <typename OStream> friend OStream &operator<<(OStream &os, const Request &r) {
      os << "SetWheelSpeed::Request {"
         << " w1_speed=" << int(r.w1_speed) << " w2_speed=" << int(r.w2_speed)
         << " w3_speed=" << int(r.w3_speed) << " w4_speed=" << int(r.w4_speed) << "}";
      return os;
    }
  };

  static bool answer(const Request &request, Response &response, Robot *robot) {
    WheelSpeeds speeds = {.front_left = -angular_speed_from_rpm(request.w2_speed),
                          .front_right = angular_speed_from_rpm(request.w1_speed),
                          .rear_left = -angular_speed_from_rpm(request.w3_speed),
                          .rear_right = angular_speed_from_rpm(request.w4_speed)};
    robot->set_target_wheel_speeds(speeds);
    return true;
  }
};

struct SetSystemLed : Proto<0x3f, 0x33> {
  struct Request : RequestT {
    // r int: [0~255]
    // g int: [0~255]
    // b int: [0~255]
    // freq: int: [1, 10]
    // t1, t2 [ms]
    uint32_t comp_mask;
    int16_t led_mask;
    uint8_t ctrl_mode;
    uint8_t effect_mode;
    uint8_t r, g, b;
    // ignored by real robot -> always equal true
    uint8_t loop;
    int16_t t1, t2;

    Request(uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri,
            const uint8_t *buffer)
        : RequestT(_sender, _receiver, _seq_id, _attri) {
      comp_mask = read<uint32_t>(buffer);
      led_mask = read<int16_t>(buffer + 4);
      ctrl_mode = buffer[6] >> 4;
      effect_mode = buffer[6] & 0xF;
      r = buffer[7];
      g = buffer[8];
      b = buffer[9];
      loop = buffer[10];
      t1 = read<int16_t>(buffer + 11);
      t2 = read<int16_t>(buffer + 13);
    }

    template <typename OStream> friend OStream &operator<<(OStream &os, const Request &r) {
      os << "SetSystemLed::Request {"
         << " comp_mask=" << std::hex << r.comp_mask << " led_mask=" << r.led_mask
         << " ctrl_mode=" << std::dec << int(r.ctrl_mode) << " effect_mode=" << int(r.effect_mode)
         << " r=" << int(r.r) << " g=" << int(r.g) << " b=" << int(r.b) << " loop=" << int(r.loop)
         << " t1=" << int(r.t1) << " t2=" << int(r.t2) << "}";
      return os;
    }
  };

  static bool answer(const Request &request, Response &response, Robot *robot) {
    // The real robot ignore the loop parameter and set it to true.
    uint8_t mask = (0xFF & request.led_mask & request.comp_mask);
    Color color = {request.r / 255.0f, request.g / 255.0f, request.b / 255.0f};
    robot->set_led_effect(color, mask, ActiveLED::LedEffect(request.effect_mode),
                          request.t1 * 0.001, request.t2 * 0.001, true || request.loop);
    return true;
  }
};

struct PlaySound : Proto<0x3f, 0xb3> {
  struct Request : RequestT {
    uint8_t action_id;
    uint8_t task_ctrl;
    uint8_t push_freq;
    uint32_t sound_id;
    uint8_t play_ctrl;
    uint16_t interval;
    uint8_t play_times;

    Request(uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri,
            const uint8_t *buffer)
        : RequestT(_sender, _receiver, _seq_id, _attri) {
      action_id = buffer[0];
      task_ctrl = buffer[1] & 0x3;
      push_freq = buffer[1] >> 2;
      sound_id = read<uint32_t>(buffer + 2);
      play_ctrl = buffer[6];
      interval = read<uint16_t>(buffer + 7);
      play_times = buffer[9];
    }

    template <typename OStream> friend OStream &operator<<(OStream &os, const Request &r) {
      os << "SetSystemLed::Request {"
         << " action_id=" << (int)r.action_id << " task_ctrl=" << (int)r.task_ctrl
         << " push_freq=" << (int)r.push_freq << " sound_id=" << r.sound_id
         << " play_ctrl=" << (int)r.play_ctrl << " interval=" << (int)r.interval
         << " play_times=" << (int)r.play_times << "}";
      return os;
    }
  };

  struct Response : ResponseT {
    bool accept;

    std::vector<uint8_t> encode() { return {0, accept}; }
    using ResponseT::ResponseT;
  };

  static bool answer(const Request &request, Response &response, Robot *robot, Commands *cmd) {
    if (request.play_ctrl == 1) {
      auto push = std::make_unique<PlaySoundPush::Response>(request);
      push->is_ack = false;
      push->need_ack = 0;
      push->seq_id = 0;
      auto a = std::make_unique<PlaySoundActionSDK>(
          cmd, request.action_id, static_cast<float>(request.push_freq), std::move(push), robot,
          request.sound_id, request.play_times);
      response.accept = accept_code(robot->submit_action(std::move(a)));
      return true;
    } else {
      // Cancel (not implemented in client yet)
      spdlog::warn("Cancel action not implemented yet");
      return true;
    }
    response.accept = true;
    return true;
  }
};

// Action
struct PositionMove : Proto<0x3f, 0x25> {
  struct Request : RequestT {
    uint8_t action_id;
    uint8_t action_ctrl;  // const = 0 in client
    uint8_t freq;
    uint8_t ctrl_mode;  // const = 0 in client
    uint8_t axis_mode;  // const = 0 in client
    int16_t pos_x;
    int16_t pos_y;
    int16_t pos_z;
    uint8_t vel_xy_max;
    int16_t agl_omg_max;

    Request(uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri,
            const uint8_t *buffer)
        : RequestT(_sender, _receiver, _seq_id, _attri) {
      action_id = buffer[0];
      action_ctrl = buffer[1] & 0x3;
      freq = buffer[1] >> 2;
      ctrl_mode = buffer[2];
      axis_mode = buffer[3];
      pos_x = read<int16_t>(buffer + 4);
      pos_y = read<int16_t>(buffer + 6);
      pos_z = read<int16_t>(buffer + 8);
      vel_xy_max = buffer[10];
      agl_omg_max = read<int16_t>(buffer + 11);
    }

    template <typename OStream> friend OStream &operator<<(OStream &os, const Request &r) {
      os << "PositionMove::Request {"
         << " action_id=" << (int)r.action_id << " action_ctrl=" << (int)r.action_ctrl
         << " freq=" << (int)r.freq << " ctrl_mode=" << (int)r.ctrl_mode
         << " axis_mode=" << (int)r.axis_mode << " pos_x=" << (int)r.pos_x
         << " pos_y=" << (int)r.pos_y << " pos_z=" << (int)r.pos_z
         << " vel_xy_max=" << (int)r.vel_xy_max << " agl_omg_max=" << (int)r.agl_omg_max << "}";
      return os;
    }

    inline Pose2D pose() const { return {pos_x / 100.0f, -pos_y / 100.0f, deg2rad(pos_z / 10.0f)}; }

    inline float linear_speed() const { return (vel_xy_max + 70.0f) / 160.0f; }

    inline float angular_speed() const { return deg2rad(agl_omg_max / 10.0f); }
  };

  struct Response : ResponseT {
    bool accept;

    std::vector<uint8_t> encode() { return {0, accept}; }
    using ResponseT::ResponseT;
  };

  static bool answer(const Request &request, Response &response, Robot *robot, Commands *cmd) {
    // DONE(jerome): implement after actions are added
    // spdlog::warn("PositionMove answer not implemented");
    // response.accept = true;

    // Start
    if (request.action_ctrl == 0) {
      auto push = std::make_unique<PositionPush::Response>(request);
      push->is_ack = false;
      push->need_ack = 0;
      push->seq_id = 0;
      auto a = std::make_unique<MoveActionSDK>(
          cmd, request.action_id, static_cast<float>(request.freq), std::move(push), robot,
          request.pose(), request.linear_speed(), request.angular_speed());
      response.accept = accept_code(robot->submit_action(std::move(a)));
      return true;
    } else {
      // Cancel (not implemented in client yet)
      spdlog::warn("Cancel action not implemented yet");
      return true;
    }
  }
};

struct ChassisPwmPercent : Proto<0x3f, 0x3c> {
  struct Request : RequestT {
    uint8_t mask;
    // [0,100], percentage
    int16_t pwm_1, pwm_2, pwm_3, pwm_4, pwm_5, pwm_6;

    Request(uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri,
            const uint8_t *buffer)
        : RequestT(_sender, _receiver, _seq_id, _attri) {
      mask = buffer[0];
      pwm_1 = read<int16_t>(buffer + 1);
      pwm_2 = read<int16_t>(buffer + 3);
      pwm_3 = read<int16_t>(buffer + 5);
      pwm_4 = read<int16_t>(buffer + 7);
      pwm_5 = read<int16_t>(buffer + 9);
      pwm_6 = read<int16_t>(buffer + 11);
    }

    template <typename OStream> friend OStream &operator<<(OStream &os, const Request &r) {
      os << "ChassisPwmPercent::Request {"
         << " mask=" << (int)r.mask << " pwm_1=" << (int)r.pwm_1 << " pwm_2=" << (int)r.pwm_2
         << " pwm_3=" << (int)r.pwm_3 << " pwm_4=" << (int)r.pwm_4 << " pwm_5=" << (int)r.pwm_5
         << " pwm_6=" << (int)r.pwm_6 << "}";
      return os;
    }
  };

  struct Response : ResponseT {
    using ResponseT::ResponseT;
  };

  static bool answer(const Request &request, Response &response, Robot *robot) {
    spdlog::info("ChassisPwmPercent ignored");
    return true;
  }
};

struct ChassisPwmFreq : Proto<0x3f, 0x2b> {
  struct Request : RequestT {
    uint8_t mask;
    uint16_t pwm_1, pwm_2, pwm_3, pwm_4, pwm_5, pwm_6;

    Request(uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri,
            const uint8_t *buffer)
        : RequestT(_sender, _receiver, _seq_id, _attri) {
      mask = buffer[0];
      pwm_1 = read<uint16_t>(buffer + 1);
      pwm_2 = read<uint16_t>(buffer + 3);
      pwm_3 = read<uint16_t>(buffer + 5);
      pwm_4 = read<uint16_t>(buffer + 7);
      pwm_5 = read<uint16_t>(buffer + 9);
      pwm_6 = read<uint16_t>(buffer + 11);
    }

    template <typename OStream> friend OStream &operator<<(OStream &os, const Request &r) {
      os << "ChassisPwmPercent::Request {"
         << " mask=" << (int)r.mask << " pwm_1=" << (int)r.pwm_1 << " pwm_2=" << (int)r.pwm_2
         << " pwm_3=" << (int)r.pwm_3 << " pwm_4=" << (int)r.pwm_4 << " pwm_5=" << (int)r.pwm_5
         << " pwm_6=" << (int)r.pwm_6 << "}";
      return os;
    }
  };

  struct Response : ResponseT {
    using ResponseT::ResponseT;
  };

  static bool answer(const Request &request, Response &response, Robot *robot) {
    spdlog::warn("ChassisPwmFreq ignored");
    return true;
  }
};

struct SetRobotMode : Proto<0x3f, 0x46> {
  struct Request : RequestT {
    uint8_t mode;

    Request(uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri,
            const uint8_t *buffer)
        : RequestT(_sender, _receiver, _seq_id, _attri) {
      mode = buffer[0];
    }

    template <typename OStream> friend OStream &operator<<(OStream &os, const Request &r) {
      os << "SetRobotMode::Request {"
         << " mode=" << int(r.mode) << " }";
      return os;
    }
  };

  static bool answer(const Request &request, Response &response, Robot *robot) {
    robot->set_mode(Robot::Mode(request.mode));
    return true;
  }
};

struct GetRobotMode : Proto<0x3f, 0x47> {
  struct Request : RequestT {
    Request(uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri,
            const uint8_t *buffer)
        : RequestT(_sender, _receiver, _seq_id, _attri) {}

    template <typename OStream> friend OStream &operator<<(OStream &os, const Request &r) {
      os << "GetRobotMode::Request { }";
      return os;
    }
  };

  struct Response : ResponseT {
    uint8_t mode;
    std::vector<uint8_t> encode() { return {0, mode}; }
    using ResponseT::ResponseT;
  };

  static bool answer(const Request &request, Response &response, Robot *robot) {
    response.mode = robot->get_mode();
    return true;
  }
};

struct VisionDetectEnable : Proto<0xa, 0xa3> {
  struct Request : RequestT {
    uint16_t type;

    Request(uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri,
            const uint8_t *buffer)
        : RequestT(_sender, _receiver, _seq_id, _attri) {
      type = read<uint16_t>(buffer);
    }

    template <typename OStream> friend OStream &operator<<(OStream &os, const Request &r) {
      os << "VisionDetectEnable::Request {"
         << " type=" << int(r.type) << " }";
      return os;
    }
  };

  static bool answer(const Request &request, Response &response, Robot *robot, Commands *cmd) {
    // spdlog::warn("VisionDetectEnable {} not implemented yet", request.type);
    robot->set_enable_vision(request.type);
    cmd->set_vision_request(request.sender, request.receiver, request.type);
    return true;
  }
};

// TODO(Jerome):  _cmdtype = DUSS_MB_TYPE_PUSH
struct ChassisSpeedMode : Proto<0x3f, 0x21> {
  struct Request : RequestT {
    // [-3.5,3.5]，x-velocity m/s
    float x_spd;
    // [-3.5,3.5]，y-velocity m/s
    float y_spd;
    // [-600,600]，theta-velocity degrees/s
    float z_spd;

    Request(uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri,
            const uint8_t *buffer)
        : RequestT(_sender, _receiver, _seq_id, _attri) {
      x_spd = read<float>(buffer);
      y_spd = read<float>(buffer + 4);
      z_spd = read<float>(buffer + 8);
    }

    template <typename OStream> friend OStream &operator<<(OStream &os, const Request &r) {
      os << "ChassisSpeedMode::Request {"
         << " x=" << (r.x_spd) << " y=" << (r.y_spd) << " z=" << (r.z_spd) << " }";
      return os;
    }
  };

  static bool answer(const Request &request, Response &response, Robot *robot) {
    Twist2D value{request.x_spd, -request.y_spd, -deg2rad(request.z_spd)};
    robot->set_target_velocity(value);
    // TODO(jerome): check if it's a bug that it send a request that does not require an ack
    response.is_ack = true;
    response.need_ack = 0;
    return true;
  }
};

struct SdkHeartBeat : Proto<0x3f, 0xd5> {
  struct Request : RequestT {
    Request(uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri,
            const uint8_t *buffer)
        : RequestT(_sender, _receiver, _seq_id, _attri) {}

    template <typename OStream> friend OStream &operator<<(OStream &os, const Request &r) {
      os << "SdkHeartBeat::Request";
      return os;
    }
  };

  static bool answer(const Request &request, Response &response, Robot *robot) { return true; }
};

struct SetSdkMode : Proto<0x3f, 0xd1> {
  struct Request : RequestT {
    uint8_t enable;

    Request(uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri,
            const uint8_t *buffer)
        : RequestT(_sender, _receiver, _seq_id, _attri) {
      enable = buffer[0];
    }

    template <typename OStream> friend OStream &operator<<(OStream &os, const Request &r) {
      os << "SetSdkMode::Request {"
         << " enable=" << bool(r.enable) << " }";
      return os;
    }
  };

  static bool answer(const Request &request, Response &response, Robot *robot, Commands *cmd) {
    robot->set_enable_sdk(request.enable);
    cmd->set_enable_sdk(request.enable);
    return true;
  }
};

struct SubscribeAddNode : Proto<0x48, 0x01> {
  struct Request : RequestT {
    uint8_t node_id;
    uint32_t sub_vision;

    Request(uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri,
            const uint8_t *buffer)
        : RequestT(_sender, _receiver, _seq_id, _attri) {
      node_id = buffer[0];
      sub_vision = read<uint32_t>(buffer + 1);
    }

    template <typename OStream> friend OStream &operator<<(OStream &os, const Request &r) {
      os << "SubscribeAddNode::Request {"
         << " node_id=" << int(r.node_id) << " sub_vision=0x" << std::hex << unsigned(r.sub_vision)
         << " }";
      os << std::dec;
      return os;
    }
  };

  struct Response : ResponseT {
    uint8_t pub_node_id;
    std::vector<uint8_t> encode() { return {0, pub_node_id}; }
    using ResponseT::ResponseT;
  };

  static bool answer(const Request &request, Response &response, Robot *robot, Commands *cmd) {
    // DONE(jerome): implement after publishers are added
    // spdlog::warn("Answer to {} not implemented yet", request);
    cmd->add_subscriber_node(request.node_id);
    return true;
  }
};

struct SubNodeReset : Proto<0x48, 0x02> {
  struct Request : RequestT {
    uint8_t node_id;

    Request(uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri,
            const uint8_t *buffer)
        : RequestT(_sender, _receiver, _seq_id, _attri) {
      node_id = buffer[0];
    }

    template <typename OStream> friend OStream &operator<<(OStream &os, const Request &r) {
      os << "SubNodeReset::Request {"
         << " node_id=" << int(r.node_id) << " }";
      return os;
    }
  };

  static bool answer(const Request &request, Response &response, Robot *robot, Commands *cmd) {
    // FONE(jerome): implement after publishers are added
    // Are nodes just for subscribers?
    // spdlog::warn("Answer to {} not implemented yet", request);
    cmd->reset_subscriber_node(request.node_id);
    return true;
  }
};

struct GripperCtrl : Proto<0x33, 0x11> {
  struct Request : RequestT {
    uint8_t id;
    // 0 pause, 1 open, 2 close
    uint8_t control;
    // [1, 100] percent of max power
    uint16_t power;

    Request(uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri,
            const uint8_t *buffer)
        : RequestT(_sender, _receiver, _seq_id, _attri) {
      id = buffer[0];
      control = buffer[1];
      power = read<uint16_t>(buffer + 2);
    }

    template <typename OStream> friend OStream &operator<<(OStream &os, const Request &r) {
      os << "GripperCtrl::Request {"
         << " id=" << (int)r.id << " control=" << (int)r.control << " power=" << (int)r.power
         << " }";
      return os;
    }
  };

  static bool answer(const Request &request, Response &response, Robot *robot) {
    // id is ignored
    robot->set_target_gripper(Robot::GripperStatus(request.control),
                              static_cast<float>(100.0 / request.power));
    return true;
  }
};

/* Not used
struct ChassisSetWorkMode : Proto<0x3f, 0x19>
{
  struct Request : RequestT {

    uint8_t mode;

    Request (uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri, const uint8_t *
buffer) : RequestT(_sender, _receiver, _seq_id, _attri)
    {
      mode = buffer[0];
    };

    template<typename OStream>
    friend OStream& operator<<(OStream& os, const Request& r)
    {
      os << "ChassisSetWorkMode::Request {"
         << " mode=" << (int)r.mode
         << " }";
      return os;
    }
  };

};
*/

struct RoboticArmMoveCtrl : Proto<0x3f, 0xb5> {
  struct Request : RequestT {
    uint8_t action_id;
    uint8_t freq;
    uint8_t action_ctrl;
    uint8_t id;
    // mode = 1 -> absolute, mode = 0 -> relative
    uint8_t mode;
    // always 0x3
    uint8_t mask;
    // front [mm]
    int32_t x;
    // up [mm]
    int32_t y;
    // always 0
    int32_t z;

    Request(uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri,
            const uint8_t *buffer)
        : RequestT(_sender, _receiver, _seq_id, _attri) {
      action_id = buffer[0];
      action_ctrl = buffer[1] & 0x3;
      freq = buffer[1] >> 2;
      id = buffer[2];
      mode = buffer[3];
      mask = buffer[4];
      x = read<int32_t>(buffer + 5);
      y = read<int32_t>(buffer + 9);
      z = read<int32_t>(buffer + 13);
    }

    template <typename OStream> friend OStream &operator<<(OStream &os, const Request &r) {
      os << "RoboticArmMoveCtrl::Request {"
         << " action_id=" << (int)r.action_id << " action_ctrl=" << (int)r.action_ctrl
         << " freq=" << (int)r.freq << " id=" << (int)r.id << " mode=" << (int)r.mode
         << " mask=" << (int)r.mask << " x=" << r.x << " y=" << r.y << " z=" << r.z << " }";
      return os;
    }
  };

  struct Response : ResponseT {
    uint8_t accept;

    std::vector<uint8_t> encode() { return {0, accept}; }
    using ResponseT::ResponseT;
  };

  static bool answer(const Request &request, Response &response, Robot *robot, Commands *cmd) {
    // DONE(jerome): implement after actions are added
    // spdlog::warn("RoboticArmMoveCtrl answer not implemented");
    // return false;

    if (request.action_ctrl == 0) {
      auto push = std::make_unique<RoboticArmMovePush::Response>(request);
      push->is_ack = false;
      push->need_ack = 0;
      push->seq_id = 0;
      // spdlog::info("Creating MoveArmAction");
      auto a = std::make_unique<MoveArmActionSDK>(
          cmd, request.action_id, static_cast<float>(request.freq), std::move(push), robot,
          request.x * 0.001, request.y * 0.001, static_cast<bool>(request.mode));
      // spdlog::info("Submitting MoveArmAction");
      response.accept = accept_code(robot->submit_action(std::move(a)));
      // spdlog::info("Action accepted? {}", response.accept);
      return true;
    } else {
      // Cancel (not implemented in client yet)
      spdlog::warn("Cancel action not implemented yet");
      return true;
    }
  }
};

struct StreamCtrl : Proto<0x3f, 0xd2> {
  enum Resolution : uint8_t {
    R720p = 0,
    R360p = 1,
    R540p = 2,
  };

  struct Request : RequestT {
    // 1 ?
    // 2 video
    // 3 audio
    uint8_t ctrl;
    // 0 wifi
    // 1 usb
    uint8_t conn_type;
    // 0 stop
    // 1 start
    uint8_t state;
    Resolution resolution;

    Request(uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri,
            const uint8_t *buffer)
        : RequestT(_sender, _receiver, _seq_id, _attri) {
      ctrl = buffer[0];
      conn_type = buffer[1] >> 4;
      state = buffer[1] & 0xF;
      resolution = Resolution(buffer[2]);
    }

    template <typename OStream> friend OStream &operator<<(OStream &os, const Request &r) {
      os << "StreamCtrl::Request {"
         << " ctrl=" << (int)r.ctrl << " conn_type=" << (int)r.conn_type
         << " state=" << (int)r.state << " resolution=" << (int)r.resolution << " }";
      return os;
    }
  };

  static bool answer(const Request &request, Response &response, Robot *robot, Commands *cmd) {
    // DONE(jerome): implement after video is added
    // spdlog::warn("StreamCtrl answer not implemented");
    if (request.ctrl == 2) {
      if (request.state) {
        if (request.resolution == R720p) {
          robot->start_streaming(1280, 720);
        } else if (request.resolution == R540p) {
          robot->start_streaming(960, 540);
        } else {
          robot->start_streaming(640, 360);
        }
        auto camera = robot->get_camera();
        if (camera->streaming) {
          auto address = cmd->sender_endpoint().address();
          cmd->get_video_streamer()->start(address, camera->width, camera->height, camera->fps);
        } else {
          spdlog::warn("Failed to start streaming");
        }
      } else {
        robot->stop_streaming();
        cmd->get_video_streamer()->stop();
      }
    }
    return true;
  }
};

struct VisionDetectStatus : Proto<0xa, 0xa5> {
  struct Request : RequestT {
    template <typename OStream> friend OStream &operator<<(OStream &os, const Request &r) {
      os << "VisionDetectStatus::Request {}";
      return os;
    }

    Request(uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri,
            const uint8_t *buffer)
        : RequestT(_sender, _receiver, _seq_id, _attri) {}
  };

  struct Response : ResponseT {
    uint16_t vision_type;

    std::vector<uint8_t> encode() {
      std::vector<uint8_t> buffer(3, 0);
      write<uint16_t>(buffer, 1, vision_type);
      return buffer;
    }
    using ResponseT::ResponseT;
  };

  static bool answer(const Request &request, Response &response, Robot *robot) {
    response.vision_type = robot->get_enable_vision();
    return true;
  }
};

struct VisionSetColor : Proto<0xa, 0xab> {
  struct Request : RequestT {
    uint8_t type;
    uint8_t color;

    template <typename OStream> friend OStream &operator<<(OStream &os, const Request &r) {
      os << "VisionSetColor::Request {"
         << " type=" << (int)r.type << " color=" << (int)r.color << " }";
      return os;
    }

    Request(uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri,
            const uint8_t *buffer)
        : RequestT(_sender, _receiver, _seq_id, _attri) {
      type = buffer[0];
      color = buffer[1];
    }
  };

  struct Response : ResponseT {
    using ResponseT::ResponseT;
  };

  static bool answer(const Request &request, Response &response, Robot *robot) {
    robot->set_vision_color(request.type, request.color);
    return true;
  }
};

struct SetArmorParam : Proto<0x3f, 0x7> {
  struct Request : RequestT {
    uint8_t armor_mask;
    uint16_t voice_energy_en;
    uint16_t voice_energy_ex;
    uint16_t voice_len_max;
    uint16_t voice_len_min;
    uint16_t voice_len_silence;
    uint16_t voice_peak_count;
    uint16_t voice_peak_min;
    uint16_t voice_peak_ave;
    uint16_t voice_peak_final;

    template <typename OStream> friend OStream &operator<<(OStream &os, const Request &r) {
      os << "SetArmorParam::Request {"
         << " armor_mask=" << (int)r.armor_mask << " voice_energy_en=" << (int)r.voice_energy_en
         << " voice_energy_ex=" << (int)r.voice_energy_ex
         << " voice_len_max=" << (int)r.voice_len_max << " voice_len_min=" << (int)r.voice_len_min
         << " voice_len_silence=" << (int)r.voice_len_silence
         << " voice_peak_count=" << (int)r.voice_peak_count
         << " voice_peak_min=" << (int)r.voice_peak_min
         << " voice_peak_ave=" << (int)r.voice_peak_ave
         << " voice_peak_final=" << (int)r.voice_peak_final << " }";
      return os;
    }

    Request(uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri,
            const uint8_t *buffer)
        : RequestT(_sender, _receiver, _seq_id, _attri) {
      armor_mask = buffer[0];
      voice_energy_en = read<uint16_t>(buffer + 1);
      voice_energy_ex = read<uint16_t>(buffer + 3);
      voice_len_max = read<uint16_t>(buffer + 5);
      voice_len_min = read<uint16_t>(buffer + 7);
      voice_len_silence = read<uint16_t>(buffer + 9);
      voice_peak_count = read<uint16_t>(buffer + 11);
      voice_peak_min = read<uint16_t>(buffer + 13);
      voice_peak_ave = read<uint16_t>(buffer + 15);
      voice_peak_final = read<uint16_t>(buffer + 17);
    }
  };

  struct Response : ResponseT {
    using ResponseT::ResponseT;
  };

  static bool answer(const Request &request, Response &response, Robot *robot) { return true; }
};

struct SensorGetData : Proto<0x3f, 0xf0> {
  struct Request : RequestT {
    uint8_t port;

    template <typename OStream> friend OStream &operator<<(OStream &os, const Request &r) {
      os << "SensorGetData::Request {"
         << " port=" << (int)r.port << " }";
      return os;
    }

    Request(uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri,
            const uint8_t *buffer)
        : RequestT(_sender, _receiver, _seq_id, _attri) {
      port = buffer[0];
    }
  };

  struct Response : ResponseT {
    uint16_t adc;
    uint8_t io;
    uint32_t time;
    uint8_t port;

    std::vector<uint8_t> encode() {
      std::vector<uint8_t> buffer(9, 0);
      buffer[1] = port;
      write<uint16_t>(buffer, 1, adc);
      buffer[3] = io;
      write<uint32_t>(buffer, 4, time);
      return buffer;
    }

    using ResponseT::ResponseT;
  };

  static bool answer(const Request &request, Response &response, Robot *robot) {
    // NOTE: We are not simulating the sensors, just answering the query
    // with values that mimic an unconnected sensor
    response.port = request.port;
    response.adc = 548;
    response.io = 0;
    response.time = 793503;
    return true;
  }
};

struct ChassisSerialSet : Proto<0x3f, 0xc0> {
  struct Request : RequestT {
    uint8_t baud_rate = 0;
    uint8_t data_bit = 0;
    uint8_t odd_even = 0;
    uint8_t stop_bit = 0;
    uint8_t tx_en = 0;
    uint8_t rx_en = 0;
    uint16_t rx_size = 0;
    uint16_t tx_size = 0;

    template <typename OStream> friend OStream &operator<<(OStream &os, const Request &r) {
      os << "SensorGetData::Request {"
         << " baud_rate=" << (int)r.baud_rate << " data_bit=" << (int)r.data_bit
         << " odd_even=" << (int)r.odd_even << " tx_en=" << (int)r.tx_en
         << " rx_en=" << (int)r.rx_en << " rx_size=" << (int)r.rx_size
         << " tx_size=" << (int)r.tx_size << " }";
      return os;
    }

    Request(uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri,
            const uint8_t *buffer)
        : RequestT(_sender, _receiver, _seq_id, _attri) {
      stop_bit = (buffer[0] >> 7) & 0x1;
      odd_even = (buffer[0] >> 5) & 0x3;
      data_bit = (buffer[0] >> 3) & 0x3;
      baud_rate = buffer[0] & 0x7;
      tx_en = (buffer[1] >> 1) & 1;
      rx_en = buffer[1] & 1;
      rx_size = read<uint16_t>(buffer + 2);
      tx_size = read<uint16_t>(buffer + 4);
    }
  };

  struct Response : ResponseT {
    using ResponseT::ResponseT;
  };

  static bool answer(const Request &request, Response &response, Robot *robot) {
    // NOTE: We are not simulating the UART, just answering the query
    return true;
  }
};

struct ChassisSerialMsgSend : Proto<0x3f, 0xc1> {
  struct Request : RequestT {
    uint8_t msg_len;
    uint8_t msg_type;
    const uint8_t *msg_buf;

    template <typename OStream> friend OStream &operator<<(OStream &os, const Request &r) {
      os << "ChassisSerialMsgSend::Request {"
         << " msg_type=" << (int)r.msg_type << " msg_len=" << (int)r.msg_len << " msg_buf=... }";
      return os;
    }

    Request(uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri,
            const uint8_t *buffer)
        : RequestT(_sender, _receiver, _seq_id, _attri) {
      msg_type = buffer[0];
      msg_len = read<uint16_t>(buffer + 1);
      msg_buf = buffer + 3;
    }
  };

  struct Response : ResponseT {
    using ResponseT::ResponseT;
  };

  static bool answer(const Request &request, Response &response, Robot *robot) {
    // NOTE: We are not simulating the UART, just answering the query
    spdlog::info(
        "Received {} bytes to forward to UART {:n}", request.msg_len,
        spdlog::to_hex(std::vector<uint8_t>(request.msg_buf, request.msg_buf + request.msg_len)));
    return true;
  }
};

#endif  // INCLUDE_COMMAND_MESSAGES_HPP_
