#include "spdlog/spdlog.h"

#include "streamer.hpp"
#include "utils.hpp"

// DONE(jerome): pass ip from command
#define PORT 40921
#define UDP_PORT 11111
#define VIDEO_STREAMER_ALLOW_TCP false

class TCPVideoStreamer final : public VideoStreamer {
 public:
  TCPVideoStreamer(ba::io_context *io_context, Robot *robot, std::string ip = "",
                   unsigned bitrate = DEFAULT_BITRATE);

 private:
  ba::ip::tcp::acceptor acceptor;
  ba::ip::tcp::socket tcp_socket;
  void send_buffer(ba::const_buffer &);
  void start_socket(const ba::ip::address &address);
  void stop_socket();
};

class UDPVideoStreamer final : public VideoStreamer {
 public:
  UDPVideoStreamer(ba::io_context *io_context, Robot *robot, std::string ip = "",
                   unsigned bitrate = DEFAULT_BITRATE);

 private:
  ba::ip::udp::socket udp_socket;
  ba::ip::udp::endpoint udp_endpoint;
  void send_buffer(ba::const_buffer &);
  void start_socket(const ba::ip::address &address);
  void stop_socket();
};

std::unique_ptr<VideoStreamer> VideoStreamer::create_video_streamer(ba::io_context *io_context,
                                                                    Robot *robot, std::string ip,
                                                                    bool udp, unsigned bitrate) {
  if (udp)
    return std::make_unique<UDPVideoStreamer>(io_context, robot, ip, bitrate);
  return std::make_unique<TCPVideoStreamer>(io_context, robot, ip, bitrate);
}

VideoStreamer::VideoStreamer(Robot *_robot, unsigned _bitrate)
    : active(false)
    , bitrate(_bitrate)
    , robot(_robot)
    , seq(0) {}

VideoStreamer::~VideoStreamer() {}

void VideoStreamer::do_step(float time_step) {
  auto camera = robot->get_camera();
  if (!camera->image.empty()) {
    send(camera->image.data());
  }
}

void VideoStreamer::send(uint8_t *buffer) {
  if (!active)
    return;
  auto data = encoder->encode(buffer);
  if (!data.empty()) {
    spdlog::debug("[Video] Will send frame #{} ({} bytes)", seq++, data.size());
    auto bbuffer = boost::asio::const_buffer(data.data(), data.size());
    send_buffer(bbuffer);
  }
}

void VideoStreamer::start(const ba::ip::address &address, unsigned image_width,
                          unsigned image_height, int fps) {
  spdlog::info("Start video streamer");
  encoder = std::make_unique<Encoder>(bitrate, image_width, image_height, fps);
  start_socket(address);
}

void VideoStreamer::stop() {
  spdlog::info("Stop video streamer");
  active = false;
  encoder = nullptr;
  stop_socket();
}

TCPVideoStreamer::TCPVideoStreamer(boost::asio::io_context *io_context, Robot *robot,
                                   std::string ip, unsigned _bitrate)
    : VideoStreamer(robot, _bitrate)
    , acceptor(*io_context, ip.size()
                                ? ba::ip::tcp::endpoint(ba::ip::make_address(ip), PORT)
                                : ba::ip::tcp::endpoint(ba::ip::tcp::v4(), PORT))
    , tcp_socket(*io_context) {
  spdlog::info("Creating a TCP video streamer on {} @ {} bps",
               STREAM(acceptor.local_endpoint()), bitrate);
}

void TCPVideoStreamer::send_buffer(boost::asio::const_buffer &buffer) {
  tcp_socket.async_write_some(buffer, [](boost::system::error_code ec, std::size_t bytes_sent) {});
}

void TCPVideoStreamer::start_socket(const ba::ip::address &address) {
  spdlog::info("Start listening for TCP connections");
  acceptor.async_accept([this](boost::system::error_code ec, ba::ip::tcp::socket new_socket) {
    if (!ec) {
      spdlog::info("Got connection!");
      tcp_socket = std::move(new_socket);
      active = true;
    }
  });
}

void TCPVideoStreamer::stop_socket() {}

UDPVideoStreamer::UDPVideoStreamer(boost::asio::io_context *io_context, Robot *robot,
                                   std::string ip, unsigned _bitrate)
    : VideoStreamer(robot, _bitrate)
    , udp_socket(*io_context,
                 ip.size() ? ba::ip::udp::endpoint(ba::ip::make_address(ip), UDP_PORT)
                           : ba::ip::udp::endpoint(ba::ip::udp::v4(), UDP_PORT)) {
  spdlog::info("Creating an UDP video streamer on {} @ {} bps", 
               STREAM(udp_socket.local_endpoint()), bitrate);
}

void UDPVideoStreamer::send_buffer(boost::asio::const_buffer &buffer) {
  udp_socket.async_send_to(buffer, udp_endpoint,
                           [](boost::system::error_code ec, std::size_t bytes_sent) {});
}

void UDPVideoStreamer::start_socket(const ba::ip::address &address) {
  udp_endpoint = ba::ip::udp::endpoint(address, PORT);
  active = true;
}

void UDPVideoStreamer::stop_socket() {}
