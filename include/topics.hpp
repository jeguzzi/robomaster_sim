#ifndef TOPIC_H
#define TOPIC_H

#include <boost/asio.hpp>
#include "messages.hpp"

class Commands;

struct Topic
{
  uint16_t period_ms;
  Commands * server;
  virtual std::vector<uint8_t> encode() = 0;
  virtual void update() = 0;
  boost::asio::io_context * io_context;
  std::shared_ptr<boost::asio::deadline_timer> timer;
  std::shared_ptr<AddSubMsg::Request> request;

  Topic(boost::asio::io_context * _io_context, Commands * _server, std::shared_ptr<AddSubMsg::Request> _request)
  {
    server = _server;
    io_context = _io_context;
    request = _request;
  }

  ~Topic()
  {
    stop();
  }

  void start();
  void stop();
  void publish();

};

#endif /* end of include guard: TOPIC_H */
