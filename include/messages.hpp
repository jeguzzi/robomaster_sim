#ifndef MESSAGES_H
#define MESSAGES_H

#include <iostream>
#include <cstdint>

#include "spdlog/fmt/bin_to_hex.h"
#include "spdlog/fmt/ostr.h"

#include "server.hpp"

struct SetSdkConnection : Proto<0x3f, 0xd4>
{
  struct Request : RequestT {
    uint8_t control;
    uint8_t host;
    uint8_t connection;
    uint8_t protocol;
    uint8_t ip[4];
    uint16_t port;

    Request (uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri, const uint8_t * buffer)
    : RequestT(_sender, _receiver, _seq_id, _attri)
    {
      control = buffer[0];
      host = buffer[1];
      connection = buffer[2];
      protocol = buffer[3];
      memcpy(ip, buffer + 4, 4);
      port = read<uint16_t>(buffer + 8);
    };

    template<typename OStream>
    friend OStream& operator<<(OStream& os, const Request& r)
    {
      os << "SetSdkConnection::Request {"
         << " control=" << std::dec << int(r.control)
         << " host=" << int(r.host)
         << " connection=" << int(r.connection)
         << " protocol=" << int(r.protocol)
         << " ip=" << int(r.ip[0]) << "." << int(r.ip[1]) << "." << int(r.ip[2]) << "." << int(r.ip[3])
         << " port=" << int(r.port)
         << " }";
      return os;
    }
  };

  struct Response : ResponseT{
    uint8_t ip[4];
    std::vector<uint8_t> encode()
    {
      return {0, 2, ip[0], ip[1], ip[2], ip[3]};
    };
    using ResponseT::ResponseT;
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

  struct Response : ResponseT{
    std::vector<uint8_t> encode()
    {
      return {0};
    };
    using ResponseT::ResponseT;
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
         << " }\n";
      return os;
    }
  };

  struct Response : ResponseT{
    std::vector<uint8_t> encode()
    {
      return {0};
    };
    using ResponseT::ResponseT;
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
         << " }\n";
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
         << " }\n";
      return os;
    }
  };

  struct Response : ResponseT{
    std::vector<uint8_t> encode()
    {
      return {0};
    };
    using ResponseT::ResponseT;
  };
};

struct DelMsg : Proto<0x48, 0x04>
{
  struct Request : RequestT {

    uint8_t node_id;
    uint8_t msg_id;
    uint8_t sub_mode;

    Request (uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri, const uint8_t * buffer)
    : RequestT(_sender, _receiver, _seq_id, _attri)
    {
      sub_mode = buffer[0];
      node_id = buffer[1];
      msg_id = buffer[2];
    };

    template<typename OStream>
    friend OStream& operator<<(OStream& os, const Request& r)
    {
      os << "DelMsg::Request {"
         << " sub_mode=" << std::dec << int(r.sub_mode)
         << " node_id=" << int(r.node_id)
         << " msg_id=" << int(r.msg_id)
         << " }\n";
      return os;
    }
  };

  struct Response : ResponseT{
    std::vector<uint8_t> encode()
    {
      return {0};
    };
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
         << " }\n";
      return os;
    }
  };

  struct Response : ResponseT{
    std::vector<uint8_t> encode()
    {
      return {0};
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
      os << "SetRobotMode::Request {"
         << " type=" << int(r.type)
         << " }\n";
      return os;
    }
  };

  struct Response : ResponseT{
    std::vector<uint8_t> encode()
    {
      return {0};
    };
    using ResponseT::ResponseT;
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
         << " }\n";
      return os;
    }
  };

  struct Response : ResponseT{
    std::vector<uint8_t> encode()
    {
      return {0};
    };
    using ResponseT::ResponseT;
  };
};


struct AddSubMsg : Proto<0x48, 0x03>
{
  struct Request : RequestT {

    uint8_t node_id;
    uint8_t msg_id;
    uint8_t timestamp;
    uint8_t stop_when_disconnect;
    uint8_t sub_mode;
    uint8_t sub_data_num;
    std::vector<uint64_t> sub_uid_list;
    uint16_t sub_freq;

    Request (uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri, const uint8_t * buffer)
    : RequestT(_sender, _receiver, _seq_id, _attri)
    {
      node_id = buffer[0];
      msg_id = buffer[1];
      timestamp = buffer[2] & 0x1;
      stop_when_disconnect = buffer[2] & 0x2;
      sub_mode = buffer[3];
      sub_data_num = buffer[4];
      buffer = buffer + 5;
      for (size_t i = 0; i < sub_data_num; i++) {
        sub_uid_list.push_back(read<uint64_t>(buffer));
        buffer += 8;
      }
      sub_freq = read<uint16_t>(buffer);
    };

    template<typename OStream>
    friend OStream& operator<<(OStream& os, const Request& r)
    {
      os << "AddSubMsg::Request {"
         << " node_id=" << std::dec << int(r.node_id)
         << " msg_id=" << int(r.msg_id)
         << " timestamp=" << int(r.timestamp)
         << " stop_when_disconnect=" << int(r.stop_when_disconnect)
         << " sub_data_num=" << int(r.sub_data_num)
         << " sub_uid_list=[" << std::hex;

      for (size_t i = 0; i < r.sub_data_num; i++) {
        os << "0x" << long(r.sub_uid_list[i]) << ", ";
      }
      os << "] sub_freq=" << std::dec << int(r.sub_freq)
         << " }\n";
      return os;
    }
  };

  struct Response : ResponseT{
    uint8_t pub_node_id;
    std::vector<uint8_t> encode()
    {
      return {0, pub_node_id, 0, 0, 0, 0, 0, 0};
    };
    using ResponseT::ResponseT;
  };
};

// DUSS_MB_TYPE_PUSH
struct PushPeriodMsg : Proto<0x48, 0x08>
{

  struct Response : ResponseT{
    uint8_t sub_mode;
    uint8_t msg_id;
    std::vector<uint8_t> subject_data;

    Response(std::shared_ptr<AddSubMsg::Request> request)
    : ResponseT(request)
    {
      msg_id = request->msg_id;
      sub_mode = request->sub_mode;
      is_ack = 0;
      need_ack = 0;
    }

    std::vector<uint8_t> encode()
    {
      std::vector<uint8_t> buffer = subject_data;
      // std::vector<uint8_t> buffer = topic->encode();
      buffer.insert(buffer.begin(), msg_id);
      buffer.insert(buffer.begin(), sub_mode);
      return buffer;
    };
    using ResponseT::ResponseT;
  };
};


#endif /* end of include guard: MESSAGES_H */
