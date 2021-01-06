#include "encoder.hpp"


Encoder::Encoder(unsigned bitrate, unsigned width, unsigned height, int fps) {
  // uint8_t endcode[] = { 0, 0, 1, 0xb7 };
  avcodec_register_all();
  /* find the h264 encoder */
  // codec = avcodec_find_encoder(AV_CODEC_ID_H264);
  codec = avcodec_find_encoder_by_name("libx264rgb");
  if (!codec) {
      fprintf(stderr, "codec not found\n");
      return;
  }
  c = avcodec_alloc_context3(codec);
  frame = av_frame_alloc();
  pkt = av_packet_alloc();
  if (!pkt)
  {
    fprintf(stderr, "av_packet_alloc failed\n");
    return;
  }
  /* put sample parameters */
  c->bit_rate = bitrate;
 /* resolution must be a multiple of two */
  c->width = width;
  c->height = height;
  /* frames per second */
  c->time_base = (AVRational){1, fps};
  c->framerate = (AVRational){fps, 1};

  c->gop_size = 10; /* emit one intra frame every ten frames */
  c->max_b_frames=1;
  // c->pix_fmt = AV_PIX_FMT_YUV420P;
  c->pix_fmt = AV_PIX_FMT_RGB24;
  //
  /* open it */
  if (avcodec_open2(c, codec, NULL) < 0) {
      fprintf(stderr, "could not open codec\n");
      return;
  }

  frame->format = c->pix_fmt;
  frame->width  = c->width;
  frame->height = c->height;
  // avpicture_fill((AVPicture*)frame, NULL, frame->format, frame->width, frame->height);

  // int ret = av_frame_get_buffer(frame, 32);
  // if (ret < 0) {
  //     fprintf(stderr, "could not alloc the frame data\n");
  //     return;
  // }
  seq = 0;
}

std::vector<unsigned char> Encoder::encode(unsigned char * buffer) {

  av_image_fill_arrays(frame->data, frame->linesize, buffer, AV_PIX_FMT_RGB24, frame->width, frame->height, 1);

  // frame->extended_data = &buffer;
  // memcpy(frame->extended_data[0], buffer, frame->width * frame->height);

  frame->pts = seq;
  seq++;
  int ret;
  /* send the frame to the encoder */
  ret = avcodec_send_frame(c, frame);
  if (ret < 0) {
      fprintf(stderr, "error sending a frame for encoding\n");
      return {};
  }
  av_packet_unref(pkt);
  while (ret >= 0) {
      ret = avcodec_receive_packet(c, pkt);
      if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
          // fprintf(stderr, "ret error %d %d %d\n", ret, AVERROR(EAGAIN), AVERROR_EOF);
          return {};
      }
      else if (ret < 0) {
          fprintf(stderr, "error during encoding\n");
          return {};
      }
      printf("encoded frame %3" PRId64" %lld (size=%5d) %d \n", pkt->pts, pkt->dts, pkt->size, seq);
      return std::vector<unsigned char>(pkt->data, pkt->data + pkt->size);
  }
  return {};
}

Encoder::~Encoder() {
  avcodec_free_context(&c);
  av_frame_free(&frame);
  av_packet_free(&pkt);
}
