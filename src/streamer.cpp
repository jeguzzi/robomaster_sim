#include "streamer.hpp"
#include <spdlog/spdlog.h>

using boost::asio::ip::udp;
using boost::asio::ip::address;

#define IPADDRESS "127.0.0.1"
#define UDP_PORT 40921

VideoStreamer::VideoStreamer(boost::asio::io_context * io_context,
                             unsigned image_width, unsigned image_height, int fps)
: socket(*io_context, udp::endpoint(udp::v4(), 11111)), seq(0) {
  encoder = std::make_shared<Encoder>(100000, image_width, image_height, fps);
}

void VideoStreamer::send(unsigned char *buffer) {
  auto data = encoder->encode(buffer);
  if(!data.empty()) {
    unsigned long frame_seq = seq++;
    spdlog::debug("[Video] Will send frame #{} ({} bytes)", frame_seq, data.size());
    socket.async_send_to(
        boost::asio::buffer(data.data(), data.size()), udp::endpoint(address::from_string(IPADDRESS), UDP_PORT),
        [frame_seq](boost::system::error_code /*ec*/, std::size_t /*bytes_sent*/)
        {
          spdlog::debug("[Video] Sent frame #{}", frame_seq);
        });
  }
}
