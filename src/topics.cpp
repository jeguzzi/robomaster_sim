#include <memory>
#include <boost/asio.hpp>

#include "spdlog/spdlog.h"

#include "topics.hpp"
#include "command.hpp"

void Topic::start()
{
  spdlog::info("[Topic] Start publisher");
  period_ms = 1000/request->sub_freq;
  timer = std::make_shared<boost::asio::deadline_timer>(*io_context, boost::posix_time::millisec(0));
  timer->async_wait(std::bind(&Topic::publish, this));
}
void Topic::stop()
{
  spdlog::info("[Topic] Stop publisher");
  timer->cancel();
  period_ms = 0;
}

void Topic::publish()
{
  if(!period_ms) return;
  timer->expires_at(timer->expires_at() + boost::posix_time::millisec(period_ms));
  timer->async_wait(std::bind(&Topic::publish, this));
  update();
  PushPeriodMsg::Response response(request);
  response.subject_data = encode();
  auto data = response.encode_msg(PushPeriodMsg::set, PushPeriodMsg::cmd);
  server->send(data);
}
