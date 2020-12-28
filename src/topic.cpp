#include "spdlog/spdlog.h"
#include "topic.hpp"
#include "command.hpp"

void Topic::start()
{
  period_ms = 1000/request.sub_freq;
  spdlog::info("[Topic] Start topic");
}

void Topic::stop()
{
  period_ms = 0;
  spdlog::info("[Topic] Stop topic");
}

void Topic::publish()
{
  PushPeriodMsg::Response response(request);
  response.subject_data = subject_data();
  auto data = response.encode_msg(PushPeriodMsg::set, PushPeriodMsg::cmd);
  server->send(data);
}

std::vector<uint8_t> Topic::subject_data(){
  subject->update(robot);
  return subject->encode();
}
