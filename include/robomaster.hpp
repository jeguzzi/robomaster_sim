#ifndef INCLUDE_ROBOMASTER_HPP_
#define INCLUDE_ROBOMASTER_HPP_

#include <memory>
#include <string>

#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>

#include "spdlog/spdlog.h"

#include "command.hpp"
#include "connection.hpp"
#include "discovery.hpp"
#include "robot/robot.hpp"
#include "streamer.hpp"

class RoboMaster {
 public:
  explicit RoboMaster(std::shared_ptr<boost::asio::io_context> io_context, Robot *robot,
                      std::string serial_number = "RM0001", bool udp_video_stream = false,
                      unsigned video_stream_bitrate = 200000, std::string ip = "",
                      unsigned prefix_len = 0, bool enable_armor_hits = false,
                      bool enable_ir_hits = false, const std::string app_id = "");
  void spin(bool);
  void do_step(float);
  ~RoboMaster() {
    spdlog::info("Will destroy RoboMaster");
    if (t) {
      io_context->stop();
      spdlog::info("IO context stopped");
      t->join();
      spdlog::info("Thread terminated");
      delete t;
      spdlog::info("Thread deleted");
      t = nullptr;
    }
  }
  VideoStreamer *get_video_streamer() { return video.get(); }

 private:
  std::shared_ptr<boost::asio::io_context> io_context;
  Robot *robot;
  Discovery discovery;
  Connection conn;
  Commands cmds;
  std::unique_ptr<VideoStreamer> video;
  boost::thread *t;
};

#endif  // INCLUDE_ROBOMASTER_HPP_
