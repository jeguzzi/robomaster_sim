#ifndef INCLUDE_STREAMER_HPP_
#define INCLUDE_STREAMER_HPP_

#include <memory>
#include <string>

#include <boost/asio.hpp>

#include "encoder.hpp"
#include "robot/robot.hpp"

namespace ba = boost::asio;

#define DEFAULT_BITRATE 200000

class VideoStreamer {
 public:
  explicit VideoStreamer(Robot *robot, unsigned bitrate = DEFAULT_BITRATE);
  static std::unique_ptr<VideoStreamer> create_video_streamer(ba::io_context *io_context,
                                                              Robot *robot, std::string ip = "",
                                                              bool udp = false,
                                                              unsigned bitrate = DEFAULT_BITRATE);
  void send(uint8_t *buffer);
  void stop();
  void start(const ba::ip::address &address, unsigned image_width, unsigned image_height, int fps);
  void do_step(float);
  virtual ~VideoStreamer();

 protected:
  bool active;
  unsigned bitrate;

 private:
  Robot *robot;
  std::unique_ptr<Encoder> encoder;
  uint64_t seq;
  virtual void send_buffer(ba::const_buffer &) = 0;
  virtual void start_socket(const ba::ip::address &address) = 0;
  virtual void stop_socket() = 0;
};

#endif  // INCLUDE_STREAMER_HPP_
