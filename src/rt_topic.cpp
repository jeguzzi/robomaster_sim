#include "rt_topic.hpp"


void RT_Topic::start()
{
  Topic::start();
  timer = std::make_shared<boost::asio::deadline_timer>(*io_context, boost::posix_time::millisec(0));
  timer->async_wait(std::bind(&RT_Topic::cb, this));
}

void RT_Topic::stop()
{
  timer->cancel();
  Topic::stop();
}

void RT_Topic::cb()
{
  if(!period_ms) return;
  timer->expires_at(timer->expires_at() + boost::posix_time::millisec(period_ms));
  timer->async_wait(std::bind(&RT_Topic::publish, this));
  publish();
}
