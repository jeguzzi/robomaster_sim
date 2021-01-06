#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>

extern "C" {
  #include "libavcodec/avcodec.h"
  #include "libavutil/frame.h"
  #include "libavutil/imgutils.h"
}

// TODO:
// - [ ] add [TCP] socket
// - [ ] add RGB pixel interface (input to the decoder) (look at https://stackoverflow.com/questions/16667687/how-to-convert-rgb-from-yuv420p-for-ffmpeg-encoder)
// see also http://ffmpeg.org/libswscale.html
// https://github.com/FFmpeg/FFmpeg/blob/master/doc/examples/scaling_video.c
// ---> perché devo anche scalare!!! (da 720p a 360p o 540p)
// - [ ] add dummy image feeder (timer) just to test the communication
// - [ ] should this be in a separate thread?

class Encoder
{
public:
  Encoder(unsigned bitrate=400000, unsigned width=1280, unsigned height=720,
          int fps=25);

  std::vector<unsigned char> encode(unsigned char * buffer);

  ~Encoder();

private:
  const AVCodec * codec;
  AVCodecContext * c;
  AVFrame * frame;
  AVPacket * pkt;
  int seq;
};

#endif /* end of include guard: ENCODER_HPP */
