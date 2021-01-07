#include <boost/asio.hpp>
#include <spdlog/spdlog.h>

#include "connection.hpp"
#include "robomaster.hpp"
#include "command.hpp"
#include "dummy_robot.hpp"
#include <boost/thread/thread.hpp>

using rm::RoboMaster;

RoboMaster::RoboMaster(boost::asio::io_context * _io_context, Robot * robot)
: io_context(_io_context ? _io_context : new boost::asio::io_context), conn(io_context, robot, 30030), cmds(io_context, robot, 20020)
{
  spdlog::set_level(spdlog::level::info);
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
