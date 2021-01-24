#ifndef INCLUDE_SUBSCRIBER_MESSAGES_HPP_
#define INCLUDE_SUBSCRIBER_MESSAGES_HPP_

#include <cstdint>
#include <iostream>
#include <vector>

#include "spdlog/fmt/bin_to_hex.h"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"

#include "protocol.hpp"

class Commands;

struct AddSubMsg : Proto<0x48, 0x03> {
  struct Request : RequestT {
    uint8_t node_id;
    uint8_t msg_id;
    uint8_t timestamp;
    uint8_t stop_when_disconnect;
    uint8_t sub_mode;
    uint8_t sub_data_num;
    std::vector<uint64_t> sub_uid_list;
    uint16_t sub_freq;

    Request(uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri,
            const uint8_t *buffer)
        : RequestT(_sender, _receiver, _seq_id, _attri) {
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
    }

    template <typename OStream> friend OStream &operator<<(OStream &os, const Request &r) {
      os << "AddSubMsg::Request {"
         << " node_id=" << std::dec << int(r.node_id) << " msg_id=" << int(r.msg_id)
         << " timestamp=" << int(r.timestamp)
         << " stop_when_disconnect=" << int(r.stop_when_disconnect)
         << " sub_data_num=" << int(r.sub_data_num) << " sub_uid_list=[" << std::hex;

      for (size_t i = 0; i < r.sub_data_num; i++) {
        os << "0x" << std::hex << r.sub_uid_list[i] << ", ";
      }
      os << "] sub_freq=" << std::dec << int(r.sub_freq) << " }" << std::dec;
      return os;
    }
  };

  struct Response : ResponseT {
    uint8_t pub_node_id;
    std::vector<uint8_t> encode() { return {0, pub_node_id, 0, 0, 0, 0, 0, 0}; }
    using ResponseT::ResponseT;
  };

  static bool answer(const Request &request, Response &response, Robot *robot, Commands *server);
};

struct DelMsg : Proto<0x48, 0x04> {
  struct Request : RequestT {
    uint8_t node_id;
    uint8_t msg_id;
    uint8_t sub_mode;

    Request(uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri,
            const uint8_t *buffer)
        : RequestT(_sender, _receiver, _seq_id, _attri) {
      sub_mode = buffer[0];
      node_id = buffer[1];
      msg_id = buffer[2];
    }

    template <typename OStream> friend OStream &operator<<(OStream &os, const Request &r) {
      os << "DelMsg::Request {"
         << " sub_mode=" << std::dec << int(r.sub_mode) << " node_id=" << int(r.node_id)
         << " msg_id=" << int(r.msg_id) << " }";
      return os;
    }
  };

  static bool answer(const Request &request, Response &response, Robot *robot, Commands *server);
};

// DUSS_MB_TYPE_PUSH
struct PushPeriodMsg : Proto<0x48, 0x08> {
  struct Response : ResponseT {
    uint8_t sub_mode;
    uint8_t msg_id;
    std::vector<uint8_t> subject_data;

    explicit Response(const AddSubMsg::Request &request)
        : ResponseT(request) {
      msg_id = request.msg_id;
      sub_mode = request.sub_mode;
      is_ack = 0;
      need_ack = 0;
    }

    std::vector<uint8_t> encode() {
      std::vector<uint8_t> buffer = subject_data;
      // std::vector<uint8_t> buffer = topic->encode();
      buffer.insert(buffer.begin(), msg_id);
      buffer.insert(buffer.begin(), sub_mode);
      return buffer;
    }
    using ResponseT::ResponseT;
  };
};
#endif  // INCLUDE_SUBSCRIBER_MESSAGES_HPP_
