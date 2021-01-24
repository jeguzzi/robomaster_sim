// main.cpp
#include <iostream>
#include "encoder.hpp"
#include <unistd.h>
#include <vector>

struct Image {

  enum Format {
    raw = 0,
    h264 = 1
  };

  std::vector<uint8_t> buffer;
  unsigned width;
  unsigned height;
  Format format;
  inline unsigned size() {
    return 3 * width * height;
  }
};

Image generate_strip_image(unsigned i0, unsigned i1, unsigned width, unsigned height) {
  Image image;
  image.width = width;
  image.height = height;
  image.buffer = std::vector<uint8_t>(image.size(), 0);
  image.format = Image::Format::raw;
  unsigned size = image.size();
  if(i0 > i1) i1 += width;
  for (size_t i = i0; i < i1; i++)
    for (size_t j = 0; j < height; j++)
      image.buffer[(3 * (j * width + i)) % size] = 255;
  return image;
}

Image convert_to_yuv(Image &image) {
  return image;
}


const unsigned height = 360;
const unsigned width = 640;

int main(int argc, char **argv) {
  std::cout << "Welcome to the H264 encoder test" << std::endl;
  Encoder encoder(100000, width, height, 25);
  for (size_t i = 0; i < 100; i++) {
    Image raw_image = generate_strip_image(i, i + 10, width, height);
    // raw_image = convert_to_yuv(raw_image);
    auto data = encoder.encode(raw_image.buffer.data());
    if(data.size()) {
      std::cout << "Will send " << data.size() << " bytes\n";
    }
  }
  std::cout << "Goodbye" << std::endl;
  return 0;
}
