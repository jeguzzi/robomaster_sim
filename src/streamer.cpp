#include <spdlog/spdlog.h>

#include "streamer.hpp"


// TODO(jerome): pass ip from command
#define IPADDRESS "192.168.1.112"
#define PORT 40921
#define UDP_PORT 11111
#define VIDEO_STREAMER_USE_UDP_

VideoStreamer::VideoStreamer(boost::asio::io_context * io_context)
:
#ifdef VIDEO_STREAMER_USE_UDP_
  udp_socket(*io_context, ba::ip::udp::endpoint(ba::ip::udp::v4(), UDP_PORT)),
  udp_endpoint(ba::ip::address::from_string(IPADDRESS), PORT),
#else
  acceptor(*io_context, ba::ip::tcp::endpoint(ba::ip::tcp::v4(), PORT)),
  tcp_socket(*io_context),
#endif
  seq(0),
  active(false) {
}

void VideoStreamer::send(unsigned char *buffer) {
  if(!active) return;
  auto data = encoder->encode(buffer);
  if(!data.empty()) {
    unsigned long frame_seq = seq++;
    frame_seq = frame_seq+1;
    spdlog::debug("[Video] Will send frame #{} ({} bytes)", frame_seq, data.size());
    auto bbuffer = boost::asio::const_buffer(data.data(), data.size());
    send_buffer(bbuffer);
  }
}

void VideoStreamer::send_buffer(boost::asio::const_buffer & buffer) {
#ifdef VIDEO_STREAMER_USE_UDP_
  udp_socket.async_send_to(buffer, udp_endpoint, [](boost::system::error_code ec, std::size_t bytes_sent) {});
#else
  tcp_socket.async_write_some(buffer, [](boost::system::error_code ec, std::size_t bytes_sent) {});
#endif
}

void VideoStreamer::start(unsigned image_width, unsigned image_height, int fps, long bitrate) {
  encoder = std::make_shared<Encoder>(bitrate, image_width, image_height, fps);
#ifdef VIDEO_STREAMER_USE_UDP_
  active = true;
#else
    spdlog::info("Start listening for TCP connections");
    acceptor.async_accept(
      [this](boost::system::error_code ec, ba::ip::tcp::socket new_socket) {
        if (!ec) {
          spdlog::info("Got connection!");
          tcp_socket = std::move(new_socket);
          active = true;
        }
      });
#endif
}

void VideoStreamer::stop() {
  active = false;
  encoder = nullptr;
}
