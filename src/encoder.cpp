#include "spdlog/spdlog.h"

#include "encoder.hpp"

extern "C" {
#include "libavutil/opt.h"
}

Encoder::Encoder(unsigned bitrate, unsigned width, unsigned height, int fps) :
  codec(nullptr), c(nullptr), frame(nullptr), pkt(nullptr), seq(0), ready(false) {
  spdlog::info("Initializing an H264 encoder with input ({}, {}), fps {} and bitrate {}", width,
               height, fps, bitrate);
  // avcodec_register_all();
  /* find the h264 encoder */
  // codec = avcodec_find_encoder(AV_CODEC_ID_H264);
  codec = avcodec_find_encoder_by_name("libx264rgb");
  // codec = avcodec_find_encoder_by_name("h264_videotoolbox");
  if (!codec) {
    spdlog::error("H264 codec not found");
    return;
  }
  c = avcodec_alloc_context3(codec);
  frame = av_frame_alloc();
  pkt = av_packet_alloc();
  if (!pkt) {
    spdlog::error("Failed to allocate av_packet");
    return;
  }

  /* put sample parameters */
  c->bit_rate = bitrate;
  c->bit_rate_tolerance = 1000000;
  /* resolution must be a multiple of two */
  c->width = width;
  c->height = height;

  // format_ctx->max_delay = 0;
  // format_ctx->probesize = 64;

  /* frames per second */
  c->time_base = AVRational{.num = 1, .den = fps};
  c->framerate = AVRational{.num = fps, .den = 1};
  c->pix_fmt = AV_PIX_FMT_RGB24;

  c->gop_size = 10; /* emit one intra frame every ten frames */
  c->max_b_frames = 1;
  // c->pix_fmt = AV_PIX_FMT_YUV420P;

  av_opt_set(c->priv_data, "preset", "superfast", 0);
  av_opt_set(c->priv_data, "tune", "zerolatency", 0);

  c->bit_rate_tolerance = 1000000;
  // c->bit_rate = bitrate;
  // c->delay = 0;
  // c->thread_count = 1;
  // c->gop_size = 0;

  // preset: ultrafast, superfast, veryfast, faster, fast,
  // medium, slow, slower, veryslow, placebo
  // av_opt_set(pCodecCtx->priv_data,"preset","slow",0);
  // tune: film, animation, grain, stillimage, psnr,
  // ssim, fastdecode, zerolatency
  // av_opt_set(pCodecCtx->priv_data,"tune","zerolatency",0);
  // profile: baseline, main, high, high10, high422, high444
  // av_opt_set(pCodecCtx->priv_data,"profile","main",0);

  //
  /* open it */
  if (avcodec_open2(c, codec, NULL) < 0) {
    spdlog::error("Could not open codec");
    return;
  }

  spdlog::info("Opened codec {}", codec->long_name);

  frame->format = c->pix_fmt;
  frame->width = c->width;
  frame->height = c->height;
  // avpicture_fill((AVPicture*)frame, NULL, frame->format, frame->width, frame->height);

  // int ret = av_frame_get_buffer(frame, 32);
  // if (ret < 0) {
  //     fprintf(stderr, "could not alloc the frame data\n");
  //     return;
  // }
  ready = true;

  // printf("Has a latency of %d frames\n", c->delay);
}

std::vector<uint8_t> Encoder::encode(uint8_t *buffer) {
  if (!ready) return {};
  av_image_fill_arrays(frame->data, frame->linesize, buffer, AV_PIX_FMT_RGB24, frame->width,
                       frame->height, 1);
  // av_image_copy(frame->data, frame->linesize, (const uint8_t **) &buffer, frame->linesize,
  // AV_PIX_FMT_RGB24, frame->width, frame->height); av_image_copy        (uint8_t *dst_data[4], int
  // dst_linesizes[4], const uint8_t *src_data[4], const int src_linesizes[4], enum AVPixelFormat
  // pix_fmt, int width, int height) av_image_fill_arrays (uint8_t *dst_data[4], int
  // dst_linesize[4],   uint8_t *src, enum AVPixelFormat pix_fmt, int width, int height, int align)
  //

  // frame->extended_data = &buffer;
  // memcpy(frame->extended_data[0], buffer, frame->width * frame->height);

  frame->pts = seq;
  seq++;
  int ret;
  /* send the frame to the encoder */
  ret = avcodec_send_frame(c, frame);
  if (ret < 0) {
    spdlog::error("error sending a frame for encoding");
    return {};
  }
  av_packet_unref(pkt);
  while (ret >= 0) {
    ret = avcodec_receive_packet(c, pkt);
    if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
      // fprintf(stderr, "ret error %d %d %d\n", ret, AVERROR(EAGAIN), AVERROR_EOF);
      return {};
    } else if (ret < 0) {
      spdlog::error("error during encoding");
      return {};
    }
    // printf("encoded frame %3" PRId64" %lld (size=%5d) %d \n", pkt->pts, pkt->dts, pkt->size,
    // seq);
    std::vector<uint8_t> buffer;
    buffer.reserve(pkt->size);
    std::copy(pkt->data, pkt->data + pkt->size, std::back_inserter(buffer));
    return buffer;
    // return std::vector<unsigned char>(pkt->data, pkt->data+pkt->size);
  }
  return {};
}

Encoder::~Encoder() {
  if (c) avcodec_free_context(&c);
  if (frame) av_frame_free(&frame);
  if (pkt) av_packet_free(&pkt);
}
