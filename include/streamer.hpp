#ifndef STREMER_HPP
#define STREMER_HPP

#include <memory>

#include <boost/asio.hpp>

#include "encoder.hpp"

using boost::asio::ip::udp;

class VideoStreamer {
public:
  VideoStreamer(boost::asio::io_context * io_context,
                unsigned image_width, unsigned image_height, int fps);
  void send(unsigned char * buffer);

private:
  std::shared_ptr<Encoder> encoder;
  udp::socket socket;
  unsigned long seq;
};

#endif /* end of include guard: STREMER_HPP */
