#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>
#include <spdlog/spdlog.h>
// #include "spdlog/cfg/env.h"

#include "robomaster.hpp"

using rm::RoboMaster;

RoboMaster::RoboMaster(boost::asio::io_context * _io_context, Robot * robot, std::string serial_number)
: io_context(_io_context ? _io_context : new boost::asio::io_context),
  discovery(io_context, serial_number), conn(io_context, robot, 30030),
  cmds(io_context, robot, 20020), video(io_context) {
  spdlog::set_level(spdlog::level::debug);
  // spdlog::cfg::load_env_levels();
  discovery.start();
  robot->set_discovery(&discovery);
  robot->set_video_streamer(&video);
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
