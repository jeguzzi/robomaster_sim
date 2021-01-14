#ifndef STREAMER_HPP
#define STREAMER_HPP

#include <memory>

#include <boost/asio.hpp>

#include "encoder.hpp"

namespace ba = boost::asio;

#define DEFAULT_BITRATE 200000

class VideoStreamer {
public:
  VideoStreamer(long bitrate=DEFAULT_BITRATE);
  static std::shared_ptr<VideoStreamer> create_video_streamer(ba::io_context * io_context, bool udp=false, long bitrate=DEFAULT_BITRATE);
  void send(unsigned char * buffer);
  void stop();
  void start(ba::ip::address & address, unsigned image_width, unsigned image_height, int fps);
protected:
  bool active;
  long bitrate;
private:
  std::shared_ptr<Encoder> encoder;
  unsigned long seq;
  virtual void send_buffer(ba::const_buffer &) = 0;
  virtual void start_socket(ba::ip::address & address) = 0;
  virtual void stop_socket() = 0;
};

#endif /* end of include guard: STREAMER_HPP */
