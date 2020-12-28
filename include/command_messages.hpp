#ifndef COMMAND_MESSAGES
#define COMMAND_MESSAGES

#include <iostream>
#include <cstdint>
#include "server.hpp"
#include "messages.hpp"

struct GetVersion : Proto<0x0, 0x1>
{
  struct Request : RequestT {

    Request (uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri, const uint8_t * buffer)
      : RequestT(_sender, _receiver, _seq_id, _attri){};

    template<typename OStream>
    friend OStream& operator<<(OStream& os, const Request& r)
    {
      os << "GetVersion::Request { }";
      return os;
    }
  };

  struct Response : ResponseT{
    uint8_t aa;
    uint8_t bb;
    uint8_t cc;
    uint8_t dd;
    std::vector<uint8_t> encode()
    {
      std::vector<uint8_t> buffer(30, 0);
      buffer[0] = aa;
      buffer[1] = bb;
      buffer[2] = cc;
      buffer[3] = dd;
      return buffer;
    };
    using ResponseT::ResponseT;
  };
};

struct GetProductVersion : Proto<0x0, 0x4f>
{
  struct Request : RequestT {

    uint8_t file_type;

    Request (uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri, const uint8_t * buffer)
      : RequestT(_sender, _receiver, _seq_id, _attri){
        file_type=buffer[0];
      };

    template<typename OStream>
    friend OStream& operator<<(OStream& os, const Request& r)
    {
      os << "GetProductVersion::Request {"
         << " file_type=" << int(r.file_type)
         << "}";
      return os;
    }
  };

  struct Response : ResponseT{
    uint8_t aa;
    uint8_t bb;
    uint16_t cc;
    std::vector<uint8_t> encode()
    {
      std::vector<uint8_t> buffer(13, 0);
      buffer[9] = aa;
      buffer[10] = bb;
      write<uint16_t>(buffer, 11, cc);
      return buffer;
    }
    using ResponseT::ResponseT;
  };
};

struct GetSn : Proto<0x0, 0x51>
{
  struct Request : RequestT {

    uint8_t type;

    Request (uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri, const uint8_t * buffer)
      : RequestT(_sender, _receiver, _seq_id, _attri){
        type = buffer[0];
      };

    template<typename OStream>
    friend OStream& operator<<(OStream& os, const Request& r)
    {
      os << "GetSn::Request {"
         << " type=" << int(r.type)
         << "}";
      return os;
    }
  };

// TODO(jerome): not sure about the string serialization (should be utf-8)
  struct Response : ResponseT{
    std::string serial_number;

    std::vector<uint8_t> encode()
    {
      std::vector<uint8_t> buffer(2 + serial_number.size(), 0);
      write<uint16_t>(buffer, 0, serial_number.size());
      for (size_t i = 0; i < serial_number.size(); i++) {
        buffer[2 + i] = (uint8_t) serial_number.data()[i];
      }
      return buffer;
    };
    using ResponseT::ResponseT;
  };
};

struct ChassisWheelSpeed : Proto<0x3f, 0x26>
{
  struct Request : RequestT {

    int8_t w1_speed;
    int8_t w2_speed;
    int8_t w3_speed;
    int8_t w4_speed;

    Request (uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri, const uint8_t * buffer)
      : RequestT(_sender, _receiver, _seq_id, _attri){
        w1_speed = buffer[0];
        w2_speed = buffer[1];
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
};

struct SetWheelSpeed : Proto<0x3f, 0x20>
{
  struct Request : RequestT {

    int8_t w1_speed;
    int8_t w2_speed;
    int8_t w3_speed;
    int8_t w4_speed;

    Request (uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri, const uint8_t * buffer)
      : RequestT(_sender, _receiver, _seq_id, _attri){
        w1_speed = buffer[0];
        w2_speed = buffer[1];
        w3_speed = buffer[2];
        w4_speed = buffer[3];
      };

    template<typename OStream>
    friend OStream& operator<<(OStream& os, const Request& r)
    {
      os << "SetWheelSpeed::Request {"
         << " w1_speed=" << int(r.w1_speed)
         << " w2_speed=" << int(r.w2_speed)
         << " w3_speed=" << int(r.w3_speed)
         << " w4_speed=" << int(r.w4_speed)
         << "}";
      return os;
    }
  };
};

struct SetSystemLed : Proto<0x3f, 0x33>
{
  struct Request : RequestT {

    uint32_t comp_mask;
    int16_t led_mask;
    uint8_t ctrl_mode;
    uint8_t effect_mode;
    uint8_t r, g, b;
    uint8_t loop;
    int16_t t1, t2;

    Request (uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri, const uint8_t * buffer)
      : RequestT(_sender, _receiver, _seq_id, _attri){
        comp_mask = read<uint32_t>(buffer);
        led_mask = read<int16_t>(buffer+4);
        ctrl_mode = buffer[6] >> 4;
        effect_mode = buffer[6] & 0xF;
        r = buffer[7];
        g = buffer[8];
        b = buffer[9];
        loop = buffer[10];
        t1 = read<int16_t>(buffer + 11);
        t2 = read<int16_t>(buffer + 13);
      };

    template<typename OStream>
    friend OStream& operator<<(OStream& os, const Request& r)
    {
      os << "SetSystemLed::Request {"
         << " comp_mask=" << std::hex << r.comp_mask
         << " led_mask=" << r.led_mask
         << " ctrl_mode=" << std::dec << int(r.ctrl_mode)
         << " effect_mode=" << int(r.effect_mode)
         << " r=" << int(r.r)
         << " g=" << int(r.g)
         << " b=" << int(r.b)
         << " loop=" << int(r.loop)
         << " t1=" << int(r.t1)
         << " t2=" << int(r.t2)
         << "}";
      return os;
    }
  };

};

struct PlaySound : Proto<0x3f, 0xb3>
{
  struct Request : RequestT {

    uint8_t action_id;
    uint8_t task_ctrl;
    uint8_t push_freq;
    uint32_t sound_id;
    uint8_t play_ctrl;
    uint16_t interval;
    uint8_t play_times;

    Request (uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri, const uint8_t * buffer)
      : RequestT(_sender, _receiver, _seq_id, _attri){
        action_id = buffer[0];
        task_ctrl = buffer[1] & 0x3;
        push_freq = buffer[1] >> 2;
        sound_id = read<uint32_t>(buffer+2);
        play_ctrl = buffer[6];
        interval = read<uint16_t>(buffer+7);
        play_times = buffer[9];
    };

    template<typename OStream>
    friend OStream& operator<<(OStream& os, const Request& r)
    {
      os << "SetSystemLed::Request {"
         << " action_id=" << (int) r.action_id
         << " task_ctrl=" << (int) r.task_ctrl
         << " push_freq=" << (int) r.push_freq
         << " sound_id=" << r.sound_id
         << " play_ctrl=" << (int) r.play_ctrl
         << " interval=" << (int) r.interval
         << " play_times=" << (int) r.play_times
         << "}";
      return os;
    }
  };

  struct Response : ResponseT{
    bool accept;

    std::vector<uint8_t> encode()
    {
      return {0, accept};
    };
    using ResponseT::ResponseT;
  };

};


struct PositionMove : Proto<0x3f, 0x25>
{
  struct Request : RequestT {

    uint8_t action_id;
    uint8_t action_ctrl;
    uint8_t freq;
    uint8_t ctrl_mode;
    uint8_t axis_mode;
    int16_t pos_x;
    int16_t pos_y;
    int16_t pos_z;
    uint8_t vel_xy_max;
    int16_t agl_omg_max;

    Request (uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri, const uint8_t * buffer)
      : RequestT(_sender, _receiver, _seq_id, _attri){
        action_id = buffer[0];
        action_ctrl = buffer[1] & 0x3;
        freq = buffer[1] >> 2;
        ctrl_mode = buffer[2];
        axis_mode = buffer[3];
        pos_x = read<int16_t>(buffer+4);
        pos_y = read<int16_t>(buffer+6);
        pos_z = read<int16_t>(buffer+8);
        vel_xy_max = buffer[10];
        agl_omg_max = read<int16_t>(buffer+11);
    };

    template<typename OStream>
    friend OStream& operator<<(OStream& os, const Request& r)
    {
      os << "PositionMove::Request {"
         << " action_id=" << (int) r.action_id
         << " action_ctrl=" << (int) r.action_ctrl
         << " freq=" << (int) r.freq
         << " ctrl_mode=" << (int) r.ctrl_mode
         << " axis_mode=" << (int) r.axis_mode
         << " pos_x=" << (int) r.pos_x
         << " pos_y=" << (int) r.pos_y
         << " pos_z=" << (int) r.pos_z
         << " vel_xy_max=" << (int) r.vel_xy_max
         << " agl_omg_max=" << (int) r.agl_omg_max
         << "}";
      return os;
    }
  };

  struct Response : ResponseT{
    bool accept;

    std::vector<uint8_t> encode()
    {
      return {0, accept};
    };
    using ResponseT::ResponseT;
  };

};


struct PositionPush : Proto<0x3f, 0x2a>
{
  struct Request : RequestT {

    uint8_t action_id;
    uint8_t percent;
    uint8_t action_state;
    int16_t pos_x;
    int16_t pos_y;
    int16_t pos_z;

    Request (uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri, const uint8_t * buffer)
      : RequestT(_sender, _receiver, _seq_id, _attri){
        action_id = buffer[0];
        percent = buffer[1];
        action_state = buffer[2];
        pos_x = read<int16_t>(buffer+3);
        pos_y = read<int16_t>(buffer+5);
        pos_z = read<int16_t>(buffer+7);
    };

    template<typename OStream>
    friend OStream& operator<<(OStream& os, const Request& r)
    {
      os << "PositionMove::Request {"
         << " action_id=" << (int) r.action_id
         << " percent=" << (int) r.percent
         << " action_state=" << (int) r.action_state
         << " pos_x=" << (int) r.pos_x
         << " pos_y=" << (int) r.pos_y
         << " pos_z=" << (int) r.pos_z
         << "}";
      return os;
    }
  };

  struct Response : ResponseT{
    uint8_t action_id;
    uint8_t percent;
    uint8_t action_state;
    int16_t pos_x;
    int16_t pos_y;
    int16_t pos_z;

    std::vector<uint8_t> encode()
    {
      std::vector<uint8_t> buffer(9, 0);
      buffer[0] = action_id;
      buffer[1] = percent;
      buffer[2] = action_state;
      write<int16_t>(buffer, 3, pos_x);
      write<int16_t>(buffer, 5, pos_y);
      write<int16_t>(buffer, 7, pos_z);
      return buffer;
    };
    using ResponseT::ResponseT;
  };

};

struct ChassisPwmPercent : Proto<0x3f, 0x3c>
{
  struct Request : RequestT {

    uint8_t mask;
    int16_t pwm_1, pwm_2, pwm_3, pwm_4, pwm_5, pwm_6;

    Request (uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri, const uint8_t * buffer)
      : RequestT(_sender, _receiver, _seq_id, _attri){
        mask = buffer[0];
        pwm_1 = read<int16_t>(buffer+1);
        pwm_2 = read<int16_t>(buffer+3);
        pwm_3 = read<int16_t>(buffer+5);
        pwm_4 = read<int16_t>(buffer+7);
        pwm_5 = read<int16_t>(buffer+9);
        pwm_6 = read<int16_t>(buffer+11);
    };

    template<typename OStream>
    friend OStream& operator<<(OStream& os, const Request& r)
    {
      os << "ChassisPwmPercent::Request {"
         << " mask=" << (int) r.mask
         << " pwm_1=" << (int) r.pwm_1
         << " pwm_2=" << (int) r.pwm_2
         << " pwm_3=" << (int) r.pwm_3
         << " pwm_4=" << (int) r.pwm_4
         << " pwm_5=" << (int) r.pwm_5
         << " pwm_6=" << (int) r.pwm_6
         << "}";
      return os;
    }
  };

  struct Response : ResponseT{
    using ResponseT::ResponseT;
  };

};

struct ChassisPwmFreq : Proto<0x3f, 0x2b>
{
  struct Request : RequestT {

    uint8_t mask;
    uint16_t pwm_1, pwm_2, pwm_3, pwm_4, pwm_5, pwm_6;

    Request (uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri, const uint8_t * buffer)
      : RequestT(_sender, _receiver, _seq_id, _attri){
        mask = buffer[0];
        pwm_1 = read<uint16_t>(buffer+1);
        pwm_2 = read<uint16_t>(buffer+3);
        pwm_3 = read<uint16_t>(buffer+5);
        pwm_4 = read<uint16_t>(buffer+7);
        pwm_5 = read<uint16_t>(buffer+9);
        pwm_6 = read<uint16_t>(buffer+11);
    };

    template<typename OStream>
    friend OStream& operator<<(OStream& os, const Request& r)
    {
      os << "ChassisPwmPercent::Request {"
         << " mask=" << (int) r.mask
         << " pwm_1=" << (int) r.pwm_1
         << " pwm_2=" << (int) r.pwm_2
         << " pwm_3=" << (int) r.pwm_3
         << " pwm_4=" << (int) r.pwm_4
         << " pwm_5=" << (int) r.pwm_5
         << " pwm_6=" << (int) r.pwm_6
         << "}";
      return os;
    }
  };

  struct Response : ResponseT{
    using ResponseT::ResponseT;
  };

};

struct SetRobotMode : Proto<0x3f, 0x46>
{
  struct Request : RequestT {

    uint8_t mode;

    Request (uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri, const uint8_t * buffer)
    : RequestT(_sender, _receiver, _seq_id, _attri)
    {
      mode = buffer[0];
    };

    template<typename OStream>
    friend OStream& operator<<(OStream& os, const Request& r)
    {
      os << "SetRobotMode::Request {"
         << " mode=" << int(r.mode)
         << " }";
      return os;
    }
  };

};

struct GetRobotMode : Proto<0x3f, 0x47>
{
  struct Request : RequestT {

    Request (uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri, const uint8_t * buffer)
    : RequestT(_sender, _receiver, _seq_id, _attri)
    {
    };

    template<typename OStream>
    friend OStream& operator<<(OStream& os, const Request& r)
    {
      os << "GetRobotMode::Request { }";
      return os;
    }
  };

  struct Response : ResponseT{
    uint8_t mode;
    std::vector<uint8_t> encode()
    {
      return {0, mode};
    };
    using ResponseT::ResponseT;
  };

};


struct VisionDetectEnable : Proto<0xa, 0xa3>
{
  struct Request : RequestT {

    uint16_t type;

    Request (uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri, const uint8_t * buffer)
    : RequestT(_sender, _receiver, _seq_id, _attri)
    {
      type = read<uint16_t>(buffer);
    };

    template<typename OStream>
    friend OStream& operator<<(OStream& os, const Request& r)
    {
      os << "VisionDetectEnable::Request {"
         << " type=" << int(r.type)
         << " }";
      return os;
    }
  };

};

// TODO(Jerome):   _cmdtype = DUSS_MB_TYPE_PUSH
struct ChassisSpeedMode : Proto<0x3f, 0x21>
{
  struct Request : RequestT {

    float x_spd;
    float y_spd;
    float z_spd;

    Request (uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri, const uint8_t * buffer)
    : RequestT(_sender, _receiver, _seq_id, _attri)
    {
      x_spd = read<float>(buffer);
      y_spd = read<float>(buffer+4);
      z_spd = read<float>(buffer+8);
    };

    template<typename OStream>
    friend OStream& operator<<(OStream& os, const Request& r)
    {
      os << "ChassisSpeedMode::Request {"
         << " x=" << (r.x_spd)
         << " y=" << (r.y_spd)
         << " z=" << (r.z_spd)
         << " }";
      return os;
    }
  };
};

struct SdkHeartBeat : Proto<0x3f, 0xd5>
{
  struct Request : RequestT {

    Request (uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri, const uint8_t * buffer)
    : RequestT(_sender, _receiver, _seq_id, _attri)
    {
    };

    template<typename OStream>
    friend OStream& operator<<(OStream& os, const Request& r)
    {
      os << "SdkHeartBeat::Request";
      return os;
    }
  };

};

struct SetSdkMode : Proto<0x3f, 0xd1>
{
  struct Request : RequestT {

    uint8_t enable;

    Request (uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri, const uint8_t * buffer)
    : RequestT(_sender, _receiver, _seq_id, _attri)
    {
      enable = buffer[0];
    };

    template<typename OStream>
    friend OStream& operator<<(OStream& os, const Request& r)
    {
      os << "SetSdkMode::Request {"
         << " enable=" << bool(r.enable)
         << " }";
      return os;
    }
  };

};

struct SubscribeAddNode : Proto<0x48, 0x01>
{
  struct Request : RequestT {

    uint8_t node_id;
    uint32_t sub_vision;

    Request (uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri, const uint8_t * buffer)
    : RequestT(_sender, _receiver, _seq_id, _attri)
    {
      node_id = buffer[0];
      sub_vision = read<uint32_t>(buffer+1);
    };

    template<typename OStream>
    friend OStream& operator<<(OStream& os, const Request& r)
    {
      os << "SubscribeAddNode::Request {"
         << " node_id=" << int(r.node_id)
         << " sub_vision=" << unsigned(r.sub_vision)
         << " }";
      return os;
    }
  };

  struct Response : ResponseT{
    uint8_t pub_node_id;
    std::vector<uint8_t> encode()
    {
      return {0, pub_node_id};
    };
    using ResponseT::ResponseT;
  };
};


struct SubNodeReset : Proto<0x48, 0x02>
{
  struct Request : RequestT {

    uint8_t node_id;

    Request (uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri, const uint8_t * buffer)
    : RequestT(_sender, _receiver, _seq_id, _attri)
    {
      node_id = buffer[0];
    };

    template<typename OStream>
    friend OStream& operator<<(OStream& os, const Request& r)
    {
      os << "SubNodeReset::Request {"
         << " node_id=" << int(r.node_id)
         << " }";
      return os;
    }
  };

};


struct GripperCtrl : Proto<0x33, 0x11>
{
  struct Request : RequestT {

    uint8_t id;
    uint8_t control;
    uint16_t power;

    Request (uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri, const uint8_t * buffer)
    : RequestT(_sender, _receiver, _seq_id, _attri)
    {
      id = buffer[0];
      control = buffer[1];
      power = read<uint16_t>(buffer + 2);
    };

    template<typename OStream>
    friend OStream& operator<<(OStream& os, const Request& r)
    {
      os << "GripperCtrl::Request {"
         << " id=" << (int)r.id
         << " control=" << (int)r.control
         << " power=" << (int)r.power
         << " }";
      return os;
    }
  };

};


struct ChassisSetWorkMode : Proto<0x3f, 0x19>
{
  struct Request : RequestT {

    uint8_t mode;

    Request (uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri, const uint8_t * buffer)
    : RequestT(_sender, _receiver, _seq_id, _attri)
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


struct RoboticArmMoveCtrl : Proto<0x3f, 0xb5>
{
  struct Request : RequestT {

    uint8_t action_id;
    uint8_t freq;
    uint8_t action_ctrl;
    uint8_t id;
    uint8_t mode;
    uint8_t mask;
    int32_t x;
    int32_t y;
    int32_t z;

    Request (uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri, const uint8_t * buffer)
    : RequestT(_sender, _receiver, _seq_id, _attri)
    {
      action_id = buffer[0];
      action_ctrl = buffer[1] & 0x3;
      freq = buffer[1] >> 2;
      id = buffer[2];
      mode = buffer[3];
      mask = buffer[4];
      x = read<int32_t>(buffer+5);
      y = read<int32_t>(buffer+9);
      z = read<int32_t>(buffer+13);
    };

    template<typename OStream>
    friend OStream& operator<<(OStream& os, const Request& r)
    {
      os << "RoboticArmMoveCtrl::Request {"
         << " action_id=" << (int)r.action_id
         << " action_ctrl=" << (int)r.action_ctrl
         << " freq=" << (int)r.freq
         << " id=" << (int)r.id
         << " mode=" << (int)r.mode
         << " mask=" << (int)r.mask
         << " x=" << r.x
         << " y=" << r.y
         << " z=" << r.z
         << " }";
      return os;
    }
  };

  struct Response : ResponseT{
    bool accept;

    std::vector<uint8_t> encode()
    {
      return {0, accept};
    };
    using ResponseT::ResponseT;
  };

};


struct RoboticArmMovePush : Proto<0x3f, 0xb6>
{
  struct Request : RequestT {

    Request (uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri, const uint8_t * buffer)
    : RequestT(_sender, _receiver, _seq_id, _attri)
    {
    };

    template<typename OStream>
    friend OStream& operator<<(OStream& os, const Request& r)
    {
      os << "RoboticArmMovePush::Request { }";
      return os;
    }
  };

  struct Response : ResponseT{

    uint8_t action_id;
    uint8_t percent;
    uint8_t action_state;
    int32_t x;
    int32_t y;

    std::vector<uint8_t> encode()
    {
      std::vector<uint8_t> buffer(11, 0);
      buffer[0] = action_id;
      buffer[1] = percent;
      buffer[2] = action_state;
      write<int32_t>(buffer, 3, x);
      write<int32_t>(buffer, 7, y);
      return buffer;
    };
    using ResponseT::ResponseT;
  };

};


struct StreamCtrl : Proto<0x3f, 0xd2>
{

  enum Resolution : uint8_t{
    R720p = 0,
    R360p = 1,
    R540p = 2,
  };

  struct Request : RequestT {

    uint8_t ctrl;
    uint8_t conn_type;
    uint8_t state;
    Resolution resolution;

    Request (uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri, const uint8_t * buffer)
    : RequestT(_sender, _receiver, _seq_id, _attri)
    {
      ctrl = buffer[0];
      conn_type = buffer[1] >> 4;
      state =  buffer[1] & 0xF;
      resolution = Resolution(buffer[2]);
    };

    template<typename OStream>
    friend OStream& operator<<(OStream& os, const Request& r)
    {
      os << "StreamCtrl::Request {"
         << " ctrl=" << (int)r.ctrl
         << " conn_type=" << (int)r.conn_type
         << " state=" << (int)r.state
         << " resolution=" << (int)r.resolution
         << " }";
      return os;
    }
  };

};

#endif /* end of include guard: COMMAND_MESSAGES */
