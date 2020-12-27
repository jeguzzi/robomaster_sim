#include <boost/asio.hpp>

#include "spdlog/spdlog.h"

#include "connection.hpp"
#include "robomaster.hpp"
#include "command.hpp"


using rm::RoboMaster;

RoboMaster::RoboMaster() {
  boost::asio::io_context io_context;
  spdlog::set_level(spdlog::level::debug);
  Connection conn = Connection(io_context, 30030);
  Commands cmds = Commands(io_context, 20020);
  io_context.run();
}
