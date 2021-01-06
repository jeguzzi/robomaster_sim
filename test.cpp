// main.cpp
#include <iostream>
#include "robomaster.hpp"

int main(int argc, char **argv) {
  std::cout << "Welcome to the robomaster simulation" << std::endl;
  boost::asio::io_context io_context;
  DummyRobot dummy(&io_context, 0.05);
  rm::RoboMaster robot(&io_context, &dummy);
  robot.spin(false);
  std::cout << "Goodbye" << std::endl;
  return 0;
};
