#include "spdlog/spdlog.h"

#include "discovery.hpp"
#include "protocol.hpp"

#define PORT 39393

Discovery::Discovery(boost::asio::io_context *io_context, std::string serial_number, std::string ip,
                     unsigned prefix_len, float period_, const std::string app_id)
    : socket(*io_context, ip.size() ? udp::endpoint(ba::ip::make_address(ip), PORT)
                                    : udp::endpoint(udp::v4(), PORT))
    , period(period_)
    , active(false) {
  socket.set_option(boost::asio::socket_base::broadcast(true));
  message = serial_number;
  // CHANGED(Jerome): comply with robomaster sending different messages on each port
  //   40927 -> serial number (like here)
  //   45678 -> an encryped string with STAConnInfo: (check[sof], pairing, ip, mac address, app id)
  // The client (SDK) listen to:
  //   45678:
  //     - `ConnectionHelper.wait_for_connection` and check that sof=0x5a5b and app_id=my_id
  //     - `scan_robot_ip` when sn is None and does not check the message
  //   40927:
  //     - `scan_robot_ip` when sn is not None and does check message = serial_number
  port = 40927;
  sta_conn_info_port = 45678;
  if (socket.local_endpoint().address().is_loopback()) {
    broadcast_address = socket.local_endpoint().address().to_v4();
  } else {
    broadcast_address =
        boost::asio::ip::network_v4(socket.local_endpoint().address().to_v4(), prefix_len)
            .broadcast();
  }
  auto ip_bytes = socket.local_endpoint().address().to_v4().to_bytes();
  sta_conn_info_message = discovery_message(false, ip_bytes, {3, 1, 3, 1, 3, 1}, app_id);
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
  udp::endpoint ep(broadcast_address, port);
  spdlog::debug("[Discovery] publish {} on {}:{}", message, ep.address().to_string(), ep.port());
  socket.async_send_to(boost::asio::buffer(message.data(), message.size() + 1), ep,
                       [](boost::system::error_code, std::size_t) {});
  udp::endpoint ep_sta(broadcast_address, sta_conn_info_port);
  spdlog::debug("[Discovery] publish {} on {}:{}",
                spdlog::to_hex(sta_conn_info_message), ep_sta.address().to_string(), ep_sta.port());
  socket.async_send_to(boost::asio::buffer(sta_conn_info_message.data(),
                        sta_conn_info_message.size()), ep_sta,
                        [](boost::system::error_code, std::size_t) {});
}

// void Discovery::publish() {
//   for (const auto port : ports) {
//     udp::endpoint ep(broadcast_address, port);
//     spdlog::debug("[Discovery] publish {} on {}:{}", message, ep.address().to_string(), ep.port());
//     socket.async_send_to(boost::asio::buffer(message.data(), message.size() + 1), ep,
//                          [](boost::system::error_code, std::size_t) {});
//   }
// }
