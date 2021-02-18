#include <boost/asio.hpp>

#include "connection.hpp"
#include "connection_messages.hpp"

using boost::asio::ip::udp;

Connection::Connection(boost::asio::io_context *io_context, Robot *robot, std::string ip,
                       unsigned short port)
    : Server(io_context, robot, ip, port) {
  register_message<SetSdkConnection>();
  spdlog::info("[Connection] Start listening on port {}", local_endpoint());
  start();
}
