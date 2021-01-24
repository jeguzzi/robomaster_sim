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
  std::unique_ptr<Subject> subject;
  float deadline;
  bool active;

  Topic(Commands * _server, Robot * _robot, AddSubMsg::Request &_request,
        std::unique_ptr<Subject> _subject)
  : server(_server), robot(_robot), request(_request), subject(std::move(_subject))
  {
  }

  virtual ~Topic() {
    stop();
  }
  void do_step(float time_step);
  virtual void start();
  virtual void stop();
  void publish();
  std::vector<uint8_t> subject_data();
};

#endif /* end of include guard: TOPIC_H */
