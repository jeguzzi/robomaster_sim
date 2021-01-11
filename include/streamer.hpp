#ifndef STREAMER_HPP
#define STREAMER_HPP

#include <memory>

#include <boost/asio.hpp>

#include "encoder.hpp"

namespace ba = boost::asio;

#define VIDEO_STREAM_USE_UDP true
#define BITRATE 100000

class VideoStreamer {
public:
  VideoStreamer(boost::asio::io_context * io_context);
  void send(unsigned char * buffer);
  void stop();
  void start(unsigned image_width, unsigned image_height, int fps, long bitrate=BITRATE);
private:
  std::shared_ptr<Encoder> encoder;
#ifdef VIDEO_STREAM_USE_UDP
  ba::ip::udp::socket udp_socket;
  ba::ip::udp::endpoint udp_endpoint;
#else
  ba::ip::tcp::acceptor acceptor;
  ba::ip::tcp::socket tcp_socket;
#endif
  unsigned long seq;
  bool active;
  void send_buffer(ba::const_buffer &);
};


#endif /* end of include guard: STREAMER_HPP */
