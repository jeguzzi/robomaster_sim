#ifndef INCLUDE_COMMAND_HPP_
#define INCLUDE_COMMAND_HPP_

#include <map>
#include <memory>
#include <string>

#include <boost/asio.hpp>

#include "server.hpp"
#include "streamer.hpp"
#include "subject.hpp"
#include "topic.hpp"

class RoboMaster;
struct VisionEvent;
struct ArmorHitEvent;
struct IRHitEvent;
struct UARTEvent;

using boost::asio::ip::udp;

class Commands : public Server {
 public:
  Commands(boost::asio::io_context *_io_context, Robot *robot, RoboMaster *rm, std::string ip = "",
           unsigned short port = 20020, bool enable_armor_hits = false,
           bool enable_ir_hits = false);
  ~Commands();
  void create_publisher(uint64_t uid, const AddSubMsg::Request &request);
  void stop_publisher(const DelMsg::Request &request);
  void do_step(float time_step);
  VideoStreamer *get_video_streamer();
  void set_vision_request(uint8_t sender, uint8_t receiver, uint16_t type);
  void set_enable_sdk(bool);
  void add_subscriber_node(uint8_t node_id);
  void reset_subscriber_node(uint8_t node_id);
  void got_heartbeat();

 private:
  using SubjectCreator = std::function<std::unique_ptr<Subject>()>;
  std::map<uint64_t, SubjectCreator> subjects;
  std::map<int, std::unique_ptr<Topic>> publishers;
  void unconnect();

  template <typename S> void register_subject() {
    subjects[S::uid] = []() -> std::unique_ptr<S> { return std::make_unique<S>(); };
  }

  RoboMaster *robomaster;
  std::unique_ptr<VisionEvent> vision_event;
  std::unique_ptr<ArmorHitEvent> armor_hit_event;
  std::unique_ptr<IRHitEvent> ir_hit_event;
  std::unique_ptr<UARTEvent> uart_event;
  bool enable_armor_hits;
  bool enable_ir_hits;
  float last_heartbeat;
  float _time;
  bool connected;
};

#endif  // INCLUDE_COMMAND_HPP__
