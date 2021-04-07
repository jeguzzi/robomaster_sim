#include "spdlog/spdlog.h"

#include "command.hpp"
#include "topic.hpp"

void Topic::do_step(float time_step) {
  // spdlog::info("[TOPIC] do_step {} {}", active, deadline);
  if (!active)
    return;
  deadline -= time_step;
  while (deadline <= 0) {
    deadline += 1.0f / request.sub_freq;
    // spdlog::info("[TOPIC] publish");
    publish();
  }
}

void Topic::start() {
  active = true;
  deadline = 0.0f;
  // deadline = 1.0f/request.sub_freq;
  spdlog::info("[Topic] Start {} @ {} Hz", subject->name(), request.sub_freq);
  // publish();
}

void Topic::stop() {
  active = false;
  spdlog::info("[Topic] Stop {}", subject->name());
}

void Topic::publish() {
  PushPeriodMsg::Response response(request);
  response.subject_data = subject_data();
  auto data = response.encode_msg(PushPeriodMsg::set, PushPeriodMsg::cmd);
  spdlog::debug("Push {} bytes: {:n}", data.size(), spdlog::to_hex(data));
  server->send(data);
}

std::vector<uint8_t> Topic::subject_data() {
  subject->update(robot);
  return subject->encode();
}
