#ifndef INCLUDE_CONNECTION_HPP_
#define INCLUDE_CONNECTION_HPP_

#include <string>

#include <boost/asio.hpp>

#include "server.hpp"

class Connection : public Server {
 public:
  Connection(boost::asio::io_context *io_context, Robot *robot, std::string ip = "",
             unsigned short port = 30030);
};

#endif  // INCLUDE_CONNECTION_HPP_
