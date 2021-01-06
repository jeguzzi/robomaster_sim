#ifndef _CONNECTION_HPP
#define _CONNECTION_HPP

#include <boost/asio.hpp>
#include "server.hpp"

class Connection : public Server {
public:
  Connection(boost::asio::io_context * io_context, Robot * robot, short port = 30030);
};

#endif /* end of include guard: _CONNECTION_HPP */
