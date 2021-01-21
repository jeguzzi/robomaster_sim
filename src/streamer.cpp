#include <spdlog/spdlog.h>

#include "streamer.hpp"


// DONE(jerome): pass ip from command
#define PORT 40921
#define UDP_PORT 11111
#define VIDEO_STREAMER_ALLOW_TCP false


class TCPVideoStreamer final : public VideoStreamer{
public:
  TCPVideoStreamer(ba::io_context * io_context, Robot * robot, long bitrate=DEFAULT_BITRATE);
private:
  ba::ip::tcp::acceptor acceptor;
  ba::ip::tcp::socket tcp_socket;
  void send_buffer(ba::const_buffer &);
  void start_socket(ba::ip::address & address);
  void stop_socket();
};

class UDPVideoStreamer final : public VideoStreamer{
public:
  UDPVideoStreamer(ba::io_context * io_context, Robot * robot, long bitrate=DEFAULT_BITRATE);
private:
  ba::ip::udp::socket udp_socket;
  ba::ip::udp::endpoint udp_endpoint;
  void send_buffer(ba::const_buffer &);
  void start_socket(ba::ip::address & address);
  void stop_socket();
};

std::shared_ptr<VideoStreamer> VideoStreamer::create_video_streamer(
    ba::io_context * io_context, Robot * robot, bool udp, long bitrate) {
  if(udp)
    return std::dynamic_pointer_cast<VideoStreamer>(
        std::make_shared<UDPVideoStreamer>(io_context, robot, bitrate));
  return std::dynamic_pointer_cast<VideoStreamer>(
        std::make_shared<TCPVideoStreamer>(io_context, robot, bitrate));
}




VideoStreamer::VideoStreamer(Robot * _robot, long _bitrate)
  :
  active(false),
  bitrate(_bitrate),
  robot(_robot),
  seq(0) {
}

void VideoStreamer::do_step(float time_step) {
  auto camera = robot->get_camera();
  if(!camera->image.empty()) {
    send(camera->image.data());
  }
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

void VideoStreamer::start(ba::ip::address & address, unsigned image_width, unsigned image_height, int fps) {
  spdlog::info("Start video streamer");
  encoder = std::make_shared<Encoder>(bitrate, image_width, image_height, fps);
  start_socket(address);
}

void VideoStreamer::stop() {
  spdlog::info("Stop video streamer");
  active = false;
  encoder = nullptr;
  stop_socket();
}


TCPVideoStreamer::TCPVideoStreamer(boost::asio::io_context * io_context, Robot * robot, long _bitrate)
:
  VideoStreamer(robot, _bitrate),
  acceptor(*io_context, ba::ip::tcp::endpoint(ba::ip::tcp::v4(), PORT)),
  tcp_socket(*io_context) {
    spdlog::info("Creating a TCP video streamer @ {} bps", bitrate);
}

void TCPVideoStreamer::send_buffer(boost::asio::const_buffer & buffer) {
  tcp_socket.async_write_some(buffer, [](boost::system::error_code ec, std::size_t bytes_sent) {});
}

void TCPVideoStreamer::start_socket(ba::ip::address & address) {
  spdlog::info("Start listening for TCP connections");
  acceptor.async_accept(
    [this](boost::system::error_code ec, ba::ip::tcp::socket new_socket) {
      if (!ec) {
        spdlog::info("Got connection!");
        tcp_socket = std::move(new_socket);
        active = true;
      }
    });
}

void TCPVideoStreamer::stop_socket() {
}



UDPVideoStreamer::UDPVideoStreamer(boost::asio::io_context * io_context, Robot * robot, long _bitrate)
  :
  VideoStreamer(robot, _bitrate),
  udp_socket(*io_context, ba::ip::udp::endpoint(ba::ip::udp::v4(), UDP_PORT)) {
    spdlog::info("Creating an UDP video streamer @ {} bps", bitrate);
}

void UDPVideoStreamer::send_buffer(boost::asio::const_buffer & buffer) {
  udp_socket.async_send_to(buffer, udp_endpoint, [](boost::system::error_code ec, std::size_t bytes_sent) {});
}

void UDPVideoStreamer::start_socket(ba::ip::address & address) {
  udp_endpoint = ba::ip::udp::endpoint(address, PORT);
  active = true;
}

void UDPVideoStreamer::stop_socket() {
}
