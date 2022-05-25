#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>

#include "spdlog/spdlog.h"

// #include "spdlog/cfg/env.h"

#include "robomaster.hpp"

#define SERIAL_NUMBER_LENGTH 14

static std::string pad_serial(std::string value) {
  std::string s = value;
  s.resize(SERIAL_NUMBER_LENGTH, '*');
  return s;
}

// DONE(not urgent): make Robot decoupled from the protocol (so that in future I can use it without
// protocol/real time/...) for that:
// - [x] set a callback (or a callback list) to be called at the end of an update step (to be set in
// the RM constructor)
// - [x] refactor actions [seems the most difficult], splitting the robot specific and the protocol
// specific part (how to link them)
// - [ ] then expose (actions, commands, and state) in lua too (first actions and commands)
// Should be able to compile robot.c in a library without the protocol (and maybe instantiate is in
// the plugin (remote_api=<bool>))

RoboMaster::RoboMaster(std::shared_ptr<boost::asio::io_context> _io_context, Robot *_robot,
                       std::string serial_number, bool udp_video_stream,
                       unsigned video_stream_bitrate, std::string ip, unsigned prefix_len,
                       bool enable_armor_hits, bool enable_ir_hits, const std::string app_id)
    : io_context(_io_context ? _io_context : std::make_shared<boost::asio::io_context>())
    , robot(_robot)
    , discovery(io_context.get(), pad_serial(serial_number), ip, prefix_len, 1.0, app_id)
    , conn(io_context.get(), robot, ip, 30030)
    , cmds(io_context.get(), robot, this, ip, 20020, enable_armor_hits, enable_ir_hits) {
  // spdlog::set_level(spdlog::level::info);
  video = VideoStreamer::create_video_streamer(io_context.get(), robot, ip, udp_video_stream,
                                               video_stream_bitrate);
  // spdlog::cfg::load_env_levels();
  discovery.start();
  robot->add_callback(std::bind(&RoboMaster::do_step, this, std::placeholders::_1));
  spdlog::info("Created a RobotMaster with serial number {}", pad_serial(serial_number));
}

void RoboMaster::do_step(float time_step) {
  discovery.do_step(time_step);
  cmds.do_step(time_step);
  if (video)
    video->do_step(time_step);
}

void RoboMaster::spin(bool thread) {
  if (thread) {
    spdlog::info("Start IO spinning in thread");
    t = new boost::thread(boost::bind(&boost::asio::io_context::run, io_context));
  } else {
    io_context->run();
  }
}
