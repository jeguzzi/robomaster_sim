#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>
#include <spdlog/spdlog.h>
// #include "spdlog/cfg/env.h"

#include "robomaster.hpp"

using rm::RoboMaster;

RoboMaster::RoboMaster(std::shared_ptr<boost::asio::io_context> _io_context, Robot * robot, std::string serial_number,
                       bool udp_video_stream, long video_stream_bitrate)
: io_context(_io_context ? _io_context : std::make_shared<boost::asio::io_context>()),
  discovery(io_context.get(), serial_number), conn(io_context.get(), robot, 30030),
  cmds(io_context.get(), robot, 20020) {
  spdlog::info("Creating a RobotMaster with serial number {}", serial_number);
  video = VideoStreamer::create_video_streamer(io_context.get(), udp_video_stream, video_stream_bitrate);
  // spdlog::cfg::load_env_levels();
  discovery.start();
  robot->set_discovery(&discovery);
  robot->set_video_streamer(video.get());
}

void RoboMaster::spin(bool thread) {
  if(thread) {
    spdlog::info("Start IO spinning in thread");
    t = new boost::thread(boost::bind(&boost::asio::io_context::run, io_context));
  }
  else {
    io_context->run();
  }
}
