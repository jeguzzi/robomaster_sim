#include <iostream>

#include "robomaster.hpp"
#include "rt_dummy_robot.hpp"
#include "spdlog/spdlog.h"

// #include "spdlog/cfg/env.h"

static void show_usage(std::string name) {
  std::cout << "Usage: " << name << " <option(s)>" << std::endl
            << "Options:" << std::endl
            << "  --help\t\t\tShow this help message" << std::endl
            << "  --log_level=<LEVEL>\t\tLog level (default: info)" << std::endl
            << "  --ip=<IP>\t\t\tRobot ip (default: 0.0.0.0)" << std::endl
            << "  --prefix_len=<LENGTH>\t\t\tRobot network prefix length (default: 0)" << std::endl
            << "  --serial_number=<SERIAL>\tRobot serial number (default: RM0001)" << std::endl
            << "  --app=<ID>\t\tThe app ID for discovery(default: '')" << std::endl
            << "  --udp\t\t\t\tVideo stream via UDP" << std::endl
            << "  --bitrate=<BITRATE>\t\tVideo stream bitrate (default: 200000)" << std::endl
            << "  --armor_hits\t\t\tPublish armor hits" << std::endl
            << "  --ir_hits\t\t\tPublish IR hits" << std::endl
            << "  --tof=<PORT>\t\t\Enable tof on a port" << std::endl
            << "  --period=<PERIOD>\t\tUpdate step [s] (default: 0.05)" << std::endl;
}

int main(int argc, char **argv) {
  std::cout << std::endl << "Welcome to the robomaster simulation" << std::endl << std::endl;
  bool use_udp = false;
  bool armor_hits = false;
  bool ir_hits = false;
  unsigned bitrate = 200000;
  char serial[100] = "RM0001";
  char log_level[100] = "info";
  char ip[100] = "";
  float period = 0.05;
  unsigned prefix_len = 0;
  unsigned tof_port;
  char app_id[8] = "";
  std::vector<unsigned> tof_ports;
  for (int i = 0; i < argc; i++) {
    if (strcmp(argv[i], "--udp") == 0) {
      use_udp = true;
      continue;
    }
    if (sscanf(argv[i], "--bitrate=%d", &bitrate)) {
      continue;
    }
    if (sscanf(argv[i], "--serial_number=%99s", serial)) {
      continue;
    }
    if (sscanf(argv[i], "--app=%7s", app_id)) {
      continue;
    }
    if (sscanf(argv[i], "--ip=%99s", ip)) {
      continue;
    }
    if (sscanf(argv[i], "--log_level=%99s", log_level)) {
      continue;
    }
    if (strcmp(argv[i], "--armor_hits") == 0) {
      armor_hits = true;
      continue;
    }
    if (strcmp(argv[i], "--ir_hits") == 0) {
      ir_hits = true;
      continue;
    }
    if (sscanf(argv[i], "--period=%f", &period)) {
      continue;
    }
    if (sscanf(argv[i], "--prefix_len=%d", &prefix_len)) {
      continue;
    }
    if (sscanf(argv[i], "--tof=%d", &tof_port)) {
      tof_ports.push_back(tof_port);
      continue;
    }
    if (strcmp(argv[i], "--help") == 0) {
      show_usage(argv[0]);
      return 0;
    }
  }

  auto io_context = std::make_shared<boost::asio::io_context>();
  spdlog::set_level(spdlog::level::from_str(log_level));
  RealTimeDummyRobot dummy(io_context.get(), period, true, true, {true, true, false}, true, true,
                           true);
  printf("app_id %s\n", app_id);
  RoboMaster robot(io_context, &dummy, std::string(serial), use_udp, bitrate, ip, prefix_len,
                   armor_hits, ir_hits, app_id);
  for (auto port : tof_ports) {
    printf("port %d\n", port);
    dummy.enable_tof(port);
  }
  spdlog::info("Start spinning");
  robot.spin(false);
  std::cout << "Goodbye" << std::endl;
  return 0;
}
