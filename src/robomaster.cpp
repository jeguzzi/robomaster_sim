#include <boost/asio.hpp>
#include <spdlog/spdlog.h>

#include "connection.hpp"
#include "robomaster.hpp"
#include "command.hpp"
#include "dummy_robot.hpp"

using rm::RoboMaster;

RoboMaster::RoboMaster() {
  boost::asio::io_context io_context;
  DummyRobot robot;
  spdlog::set_level(spdlog::level::debug);
  Connection conn = Connection(&robot, io_context, 30030);
  Commands cmds = Commands(&robot, io_context, 20020);
  io_context.run();
}
