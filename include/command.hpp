#ifndef _COMMANDS_HPP
#define _COMMANDS_HPP

#include <memory>

#include <boost/asio.hpp>

#include "server.hpp"
#include "subject.hpp"
#include "topic.hpp"
#include "streamer.hpp"


class RoboMaster;
struct VisionEvent;
struct ArmorHitEvent;

using boost::asio::ip::udp;

class Commands : public Server {
public:
  Commands(boost::asio::io_context * _io_context, Robot * robot, RoboMaster * rm, short port = 20020);
  ~Commands();
  void create_publisher(uint64_t uid, AddSubMsg::Request &request);
  void stop_publisher(DelMsg::Request &request);
  void do_step(float time_step);
  VideoStreamer * get_video_streamer();
  void set_vision_request(uint8_t sender, uint8_t receiver, uint16_t type);
  void set_enable_sdk(bool);
  void add_subscriber_node(uint8_t node_id);
  void reset_subscriber_node(uint8_t node_id);
private:

  using SubjectCreator = std::function<std::unique_ptr<Subject>()> ;
  std::map<uint64_t, SubjectCreator> subjects;
  std::map<int, std::unique_ptr<Topic>> publishers;

  template<typename S>
  void register_subject() {
    subjects[S::uid] = []() -> std::unique_ptr<S>{ return std::make_unique<S>(); };
  }

  RoboMaster * robomaster;
  std::unique_ptr<VisionEvent> vision_event;
  std::unique_ptr<ArmorHitEvent> armor_hit_event;
};

#endif /* end of include guard: _COMMANDS_HPP */
