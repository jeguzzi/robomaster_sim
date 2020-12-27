#ifndef SERVER_HPP
#define SERVER_HPP

#include <boost/asio.hpp>

#include <cstdint>
#include <vector>
#include <map>
#include <memory>
#include <optional>

#include "spdlog/spdlog.h"
#include "spdlog/fmt/bin_to_hex.h"

using boost::asio::ip::udp;

template <typename T>
T read(const uint8_t * buffer)
{
  T value;
  uint8_t * bytes = (uint8_t *)&value;
  memcpy(bytes, buffer, sizeof(T));
  return value;
}

template <typename T>
void write(std::vector<uint8_t> & buffer, short index, T value)
{
  uint8_t * bytes = (uint8_t *)&value;
  for (size_t i = 0; i < sizeof(T); i++) {
    buffer[index + i] = bytes[i];
  }
}
unsigned key_from(uint8_t _set, uint8_t _cmd);

template<uint8_t _set, uint8_t _cmd>
struct Proto {
  static inline uint8_t set = _set;
  static inline uint8_t cmd = _cmd;
  static inline int key = key_from(_set, _cmd);
};

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

  ResponseT(std::shared_ptr<RequestT> request)
  {
    is_ack = request->need_ack();
    seq_id = request->seq_id;
    sender = request->receiver;
    receiver = request->sender;
    need_ack = !is_ack;
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
    return {};
  }

  std::vector<uint8_t> encode_msg(uint8_t set, uint8_t id);
};


class Server
{

  typedef std::function<std::vector<uint8_t>(uint8_t, uint8_t, uint16_t, uint8_t, const uint8_t *)> Callback;

public:
  Server(boost::asio::io_context& io_context, short port = 30030);
  void start();
protected:
  boost::asio::io_context * io_context;

  template<typename R>
  void add_route(std::function<std::optional<typename R::Response>(std::shared_ptr<typename R::Request>)> callback)
  {
    callbacks[R::key] = [callback](uint8_t sender, uint8_t receiver, uint16_t seq_id, uint8_t attri, const uint8_t * buffer) -> std::vector<uint8_t>
    {
      auto request = std::make_shared<typename R::Request>(sender, receiver, seq_id, attri, buffer);
      spdlog::debug("Got {}", *request);
      auto response = callback(request);

      if(response)
      {
        return response->encode_msg(R::set, R::cmd);
      }
      return {};
    };
  }

  void send(std::vector<uint8_t> data);

private:
  udp::socket socket_;
  udp::endpoint sender_endpoint_;
  enum { max_length = 1024 };
  uint8_t data_[max_length];
  std::map<int, Callback> callbacks;
  std::vector<uint8_t> answer_request(const uint8_t * buffer, size_t length);
  void has_received_bytes(const uint8_t * raw_request, size_t length);
  void do_receive();
};

#endif /* end of include guard: SERVER_HPP */
