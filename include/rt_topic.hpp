#ifndef RT_TOPIC_H
#define RT_TOPIC_H

#include <boost/asio.hpp>
#include <topic.hpp>

struct RT_Topic : Topic
{
  boost::asio::io_context * io_context;
  std::shared_ptr<boost::asio::deadline_timer> timer;

  RT_Topic(Commands * _server, Robot * _robot, AddSubMsg::Request &_request,
           std::shared_ptr<Subject> _subject, boost::asio::io_context * _io_context)
  : Topic(_server, _robot, _request, _subject), io_context(_io_context)
  {
  }

  virtual void start();
  virtual void stop();
private:
  void cb();

};

#endif /* end of include guard: RT_TOPIC_H */
