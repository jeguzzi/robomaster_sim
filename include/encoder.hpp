#ifndef INCLUDE_ENCODER_HPP_
#define INCLUDE_ENCODER_HPP_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>

extern "C" {
#include "libavcodec/avcodec.h"
#include "libavutil/frame.h"
#include "libavutil/imgutils.h"
}

// DONE:
// - [x] add [TCP] socket
// - [x] add RGB pixel interface (input to the decoder) (look at
// https://stackoverflow.com/questions/16667687/how-to-convert-rgb-from-yuv420p-for-ffmpeg-encoder)
// see also http://ffmpeg.org/libswscale.html
// https://github.com/FFmpeg/FFmpeg/blob/master/doc/examples/scaling_video.c
// ---> perché devo anche scalare!!! (da 720p a 360p o 540p)
// non needed as libx264 accept RGB
// - [x] add dummy image feeder (timer) just to test the communication
// - [x] should this be in a separate thread? -> no

class Encoder {
 public:
  explicit Encoder(unsigned bitrate = 400000, unsigned width = 1280, unsigned height = 720,
                   int fps = 25);

  std::vector<uint8_t> encode(uint8_t *buffer);

  ~Encoder();

 private:
  const AVCodec *codec;
  AVCodecContext *c;
  AVFrame *frame;
  AVPacket *pkt;
  int seq;
  bool ready;
};

#endif  // INCLUDE_ENCODER_HPP_
