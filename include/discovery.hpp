#ifndef INCLUDE_DISCOVERY_HPP_
#define INCLUDE_DISCOVERY_HPP_

#include <string>
#include <vector>

#include <boost/asio.hpp>

namespace ba = boost::asio;

using ba::ip::udp;

class Discovery {
 public:
  Discovery(boost::asio::io_context *io_context, std::string serial_number, std::string ip = "",
            unsigned prefix_len = 0, float period = 1.0, const std::string app_id = "");
  void start();
  void stop();
  void do_step(float time_step);

 private:
  udp::socket socket;
  float period;
  bool active;
  float deadline;
  unsigned port;
  std::string message;
  unsigned sta_conn_info_port;
  std::vector<uint8_t> sta_conn_info_message;
  boost::asio::ip::address_v4 broadcast_address;
  void publish();
};

#endif  // INCLUDE_DISCOVERY_HPP_
