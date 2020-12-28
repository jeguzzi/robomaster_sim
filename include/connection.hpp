#ifndef _CONNECTION_HPP
#define _CONNECTION_HPP

#include <boost/asio.hpp>
#include "server.hpp"

class Connection : public Server {
public:
  Connection(Robot * robot, boost::asio::io_context& io_context, short port = 30030);
};

#endif /* end of include guard: _CONNECTION_HPP */
