#ifndef INCLUDE_CONNECTION_MESSAGES_HPP_
#define INCLUDE_CONNECTION_MESSAGES_HPP_

#include <cstdint>
#include <iostream>
#include <vector>

#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"

#include "protocol.hpp"

struct SetSdkConnection : Proto<0x3f, 0xd4> {
  struct Request : RequestT {
    uint8_t control;
    uint8_t host;
    uint8_t connection;
    uint8_t protocol;
    uint8_t ip[4];
    uint16_t port;

    Request(uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri,
            const uint8_t *buffer)
        : RequestT(_sender, _receiver, _seq_id, _attri) {
      control = buffer[0];
      host = buffer[1];
      connection = buffer[2];
      protocol = buffer[3];
      memcpy(ip, buffer + 4, 4);
      port = read<uint16_t>(buffer + 8);
    }

    template <typename OStream> friend OStream &operator<<(OStream &os, const Request &r) {
      os << "SetSdkConnection::Request {"
         << " control=" << std::dec << int(r.control) << " host=" << int(r.host)
         << " connection=" << int(r.connection) << " protocol=" << int(r.protocol)
         << " ip=" << int(r.ip[0]) << "." << int(r.ip[1]) << "." << int(r.ip[2]) << "."
         << int(r.ip[3]) << " port=" << int(r.port) << " }";
      return os;
    }
  };

  struct Response : ResponseT {
    uint8_t ip[4];
    std::vector<uint8_t> encode() { return {0, 2, ip[0], ip[1], ip[2], ip[3]}; }
    using ResponseT::ResponseT;
  };

  static bool answer(const Request &request, Response &response, Robot *robot) {
    response.ip[0] = 0;
    response.ip[1] = 0;
    response.ip[2] = 0;
    response.ip[3] = 0;
    // response.ip[0] = 127;
    // response.ip[1] = 0;
    // response.ip[2] = 0;
    // response.ip[3] = 1;
    return true;
  }
};

#endif  // INCLUDE_CONNECTION_MESSAGES_HPP_
