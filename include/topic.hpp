#ifndef TOPIC_H
#define TOPIC_H

#include <memory>
#include "subscriber_messages.hpp"
#include "subject.hpp"

class Commands;

struct Topic
{
  Commands * server;
  Robot * robot;
  AddSubMsg::Request request;
  std::shared_ptr<Subject> subject;
  unsigned period_ms;

  Topic(Commands * _server, Robot * _robot, AddSubMsg::Request &_request,
        std::shared_ptr<Subject> _subject)
  : server(_server), robot(_robot), request(_request), subject(_subject)
  {
  }

  virtual ~Topic() {
    stop();
  }

  virtual void start();
  virtual void stop();
  void publish();
  std::vector<uint8_t> subject_data();
};

#endif /* end of include guard: TOPIC_H */
