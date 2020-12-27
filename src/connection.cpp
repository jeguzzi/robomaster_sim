#include <boost/asio.hpp>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <cstdint>

#include "spdlog/spdlog.h"
#include "spdlog/fmt/ostr.h"

#include "messages.hpp"
#include "server.hpp"
#include "connection.hpp"


using boost::asio::ip::udp;

Connection::Connection(boost::asio::io_context& io_context, short port)
: Server(io_context, port)
{
  add_route<SetSdkConnection>([](std::shared_ptr<SetSdkConnection::Request> request) -> std::optional<SetSdkConnection::Response>{
    SetSdkConnection::Response response(request);
    response.ip[0] = 127;
    response.ip[1] = 0;
    response.ip[2] = 0;
    response.ip[3] = 0;
    return response;
  });
  spdlog::info("[Connection] Start listening on port {}", port);
  start();
}
