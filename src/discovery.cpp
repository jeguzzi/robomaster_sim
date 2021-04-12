#include "spdlog/spdlog.h"

#include "discovery.hpp"

#define PORT 39393

Discovery::Discovery(boost::asio::io_context *io_context, std::string serial_number, std::string ip,
                     unsigned prefix_len, float period_)
    : socket(*io_context, ip.size() ? udp::endpoint(ba::ip::address::from_string(ip), PORT)
                                    : udp::endpoint(udp::v4(), PORT))
    , period(period_)
    , active(false) {
  socket.set_option(boost::asio::socket_base::broadcast(true));
  message = serial_number;
  ports = {45678, 40927};
  broadcast_address =
      boost::asio::ip::network_v4(socket.local_endpoint().address().to_v4(), prefix_len)
          .broadcast();
}

void Discovery::start() {
  spdlog::info("[Discovery] Start broadcasting to {}", broadcast_address.to_string());
  active = true;
  deadline = period;
}

void Discovery::stop() { active = false; }

void Discovery::do_step(float time_step) {
  if (!active)
    return;
  deadline -= time_step;
  if (deadline < 0) {
    publish();
    deadline += period;
  }
}

void Discovery::publish() {
  for (const auto port : ports) {
    udp::endpoint ep(broadcast_address, port);
    spdlog::debug("[Discovery] publish {} on {}:{}", message, ep.address().to_string(), ep.port());
    socket.async_send_to(boost::asio::buffer(message.data(), message.size()), ep,
                         [](boost::system::error_code, std::size_t) {});
  }
}
