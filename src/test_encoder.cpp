#include <iostream>
#include <vector>

#include "encoder.hpp"
#include "spdlog/spdlog.h"

struct Image {
  enum Format { raw = 0, h264 = 1 };

  std::vector<uint8_t> buffer;
  unsigned width;
  unsigned height;
  Format format;
  inline unsigned size() { return 3 * width * height; }
};

Image generate_strip_image(unsigned i0, unsigned i1, unsigned width, unsigned height) {
  Image image;
  image.width = width;
  image.height = height;
  image.buffer = std::vector<uint8_t>(image.size(), 0);
  image.format = Image::Format::raw;
  unsigned size = image.size();
  if (i0 > i1)
    i1 += width;
  for (size_t i = i0; i < i1; i++)
    for (size_t j = 0; j < height; j++)
      image.buffer[(3 * (j * width + i)) % size] = 255;
  return image;
}

constexpr unsigned height = 360;
constexpr unsigned width = 640;

int main(int argc, char **argv) {
  std::cout << std::endl << "Welcome to the H264 encoder test" << std::endl << std::endl;
  Encoder encoder(100000, width, height, 25);
  for (size_t i = 0; i < 100; i++) {
    Image raw_image = generate_strip_image(i, i + 10, width, height);
    auto data = encoder.encode(raw_image.buffer.data());
    if (data.size()) {
      spdlog::info("Will send {} bytes", data.size());
    }
  }
  std::cout << std::endl << "Goodbye" << std::endl << std::endl;
  return 0;
}
