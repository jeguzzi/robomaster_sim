#ifndef _COMMANDS_HPP
#define _COMMANDS_HPP

#include <memory>

#include <boost/asio.hpp>

#include "server.hpp"
#include "rt_topic.hpp"
#include "subject.hpp"

using boost::asio::ip::udp;

class Commands : public Server {
public:
  Commands(Robot * robot, boost::asio::io_context& _io_context, short port = 20020);
  void create_publisher(uint64_t uid, AddSubMsg::Request &request);
  void stop_publisher(DelMsg::Request &request);
private:

  typedef std::function<std::shared_ptr<Subject>()> SubjectCreator;
  std::map<uint64_t, SubjectCreator> subjects;
  std::map<int, std::shared_ptr<Topic>> publishers;

  template<typename S>
  void register_subject() {
    subjects[S::uid] = []() -> std::shared_ptr<S>{ return std::make_shared<S>(); };
  }
};

#endif /* end of include guard: _COMMANDS_HPP */
