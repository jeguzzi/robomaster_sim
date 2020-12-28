#include <boost/asio.hpp>

#include "connection.hpp"
#include "connection_messages.hpp"

using boost::asio::ip::udp;

Connection::Connection(Robot * robot, boost::asio::io_context& io_context, short port)
: Server(robot, io_context, port)
{
  register_message<SetSdkConnection>();
  spdlog::info("[Connection] Start listening on port {}", port);
  start();
}
