#ifndef ROBOMASTER_HPP_
#define ROBOMASTER_HPP_

#include <string>
#include <memory>

#include <boost/thread/thread.hpp>
#include <boost/asio.hpp>
#include <spdlog/spdlog.h>

#include "connection.hpp"
#include "command.hpp"
#include "discovery.hpp"
#include "streamer.hpp"
#include "robot.hpp"

class RoboMaster {
public:
  explicit RoboMaster(std::shared_ptr<boost::asio::io_context> io_context, Robot * robot,
                      std::string serial_number="RM0001",
                      bool udp_video_stream=false, long video_stream_bitrate=200000);
  void spin(bool);
  void do_step(float);
  ~RoboMaster(){
    spdlog::info("Will destroy RoboMaster");
    if(t) {
      io_context->stop();
      spdlog::info("IO context stopped");
      t->join();
      spdlog::info("Thread terminated");
      delete t;
      spdlog::info("Thread deleted");
      t = nullptr;
    }
  }
  VideoStreamer * get_video_streamer() {
    return video.get();
  }
private:
  std::shared_ptr<boost::asio::io_context> io_context;
  Robot * robot;
  Discovery discovery;
  Connection conn;
  Commands cmds;
  std::unique_ptr<VideoStreamer> video;
  boost::thread * t;
};


#endif // ROBOMASTER_HPP_
