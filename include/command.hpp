#ifndef _COMMANDS_HPP
#define _COMMANDS_HPP

#include <boost/asio.hpp>
#include "server.hpp"
#include "topics.hpp"

using boost::asio::ip::udp;

class Commands : Server {
protected:

  friend struct Topic;

public:
  Commands(boost::asio::io_context& _io_context, short port = 20020);

private:
  std::map<uint64_t, std::shared_ptr<Topic>> topics;
  std::map<int, std::shared_ptr<Topic>> publishers;

  void create_publisher_with(uint64_t uid, std::shared_ptr<AddSubMsg::Request> request);

  template<typename T>
  std::shared_ptr<T> create_publisher(std::shared_ptr<AddSubMsg::Request> request);

  void stop_publisher(std::shared_ptr<DelMsg::Request> request);
};

#endif /* end of include guard: _COMMANDS_HPP */
