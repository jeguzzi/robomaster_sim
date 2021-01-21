#ifndef PROTOCOL_HPP
#define PROTOCOL_HPP

#include <cstdint>
#include <vector>

#include <spdlog/spdlog.h>
#include <spdlog/fmt/bin_to_hex.h>

#include "robot.hpp"
#include "utils.hpp"

struct RequestT
{
  uint8_t attri;
  uint8_t sender;
  uint8_t receiver;
  uint16_t seq_id;
  inline bool is_ack(){
    return (attri & 0x80) != 0;
  }
  inline bool need_ack(){
    return (attri & 0x60) >> 5;
  }
  RequestT(uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri)
  {
    sender = _sender;
    receiver = _receiver;
    seq_id = _seq_id;
    attri = _attri;
  }
};

struct ResponseT
{
  uint8_t is_ack;
  uint16_t seq_id;
  uint8_t sender;
  uint8_t receiver;
  uint8_t need_ack;

  ResponseT(RequestT &request)
  {
    is_ack = request.need_ack();
    seq_id = request.seq_id;
    sender = request.receiver;
    receiver = request.sender;
    need_ack = !is_ack;
  }

  ResponseT(uint8_t sender, uint8_t receiver)
  {
    sender = receiver;
    receiver = sender;
    is_ack = 0;
    need_ack = 0;
  }

  inline uint8_t attri ()
  {
    uint8_t v = 0;
    if (is_ack) v = 1 << 7;
    v += need_ack << 5;
    return v;
  }

  virtual std::vector<uint8_t> encode()
  {
    return {0};
  };

  virtual ~ResponseT(){};

  std::vector<uint8_t> encode_msg(uint8_t set, uint8_t id);
};

template<uint8_t _set, uint8_t _cmd>
struct Proto {
  static inline uint8_t set = _set;
  static inline uint8_t cmd = _cmd;
  static inline int key = key_from(_set, _cmd);

  struct Response : ResponseT{
    using ResponseT::ResponseT;
  };

  struct Request : RequestT{
    Request (uint8_t _sender, uint8_t _receiver, uint16_t _seq_id, uint8_t _attri, const uint8_t * buffer)
    : RequestT(_sender, _receiver, _seq_id, _attri){};
  };

  static bool answer(Request &request, Response &response, Robot * robot) {
    return false;
  };

};

bool decode_request(
    const uint8_t * buffer, size_t length,
    uint8_t * cmd_set, uint8_t * cmd_id, uint16_t * seq_id, uint8_t * attri,
    uint8_t * sender, uint8_t * receiver, const uint8_t * * payload);

#endif /* end of include guard: PROTOCOL_HPP */
