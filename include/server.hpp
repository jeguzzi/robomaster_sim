#ifndef SERVER_HPP
#define SERVER_HPP

#include <cstdint>
#include <vector>
#include <map>

#include <boost/asio.hpp>
#include <spdlog/spdlog.h>
#include <spdlog/fmt/bin_to_hex.h>

#include "robot.hpp"

using boost::asio::ip::udp;

class Server
{

  typedef std::function<std::vector<uint8_t>(uint8_t, uint8_t, uint16_t, uint8_t, const uint8_t *)> Callback;

public:
  Server(boost::asio::io_context * io_context, Robot * robot, short port = 30030);
  ~Server() {

  }
  void start();
  void send(std::vector<uint8_t> data);

  boost::asio::io_context * get_io_context() {
    return io_context;
  }

protected:
  boost::asio::io_context * io_context;
  Robot * robot;
  template<typename R, typename ... Args>
  void register_message(Args ... args)
  {
    Robot * r = robot;
    callbacks[R::key] = [r, args ...](uint8_t sender, uint8_t receiver, uint16_t seq_id, uint8_t attri, const uint8_t * buffer) -> std::vector<uint8_t>
    {
      typename R::Request request(sender, receiver, seq_id, attri, buffer);
      typename R::Response response(request);
      spdlog::debug("Got {} ({})", request, request.need_ack());
      bool valid = R::answer(request, response, r, args ...);
      if(valid)
      {
        return response.encode_msg(R::set, R::cmd);
      }
      return {};
    };
  }


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
