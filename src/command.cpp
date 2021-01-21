
// #include <cstdlib>
// #include <functional>
// #include <optional>
#include <cstdint>

#include <boost/asio.hpp>
// #include <boost/ref.hpp>

#include "spdlog/spdlog.h"
#include "spdlog/fmt/bin_to_hex.h"
#include "spdlog/fmt/ostr.h"

#include "command.hpp"
#include "command_messages.hpp"
#include "subscriber_messages.hpp"
#include "command_subjects.hpp"
#include "rt_topic.hpp"
#include "robomaster.hpp"
#include "event.hpp"

// #include "command_topics.hpp"

// TODO(jerome): actions
// TODO(jerome): publisher
//
using boost::asio::ip::udp;

bool AddSubMsg::answer(Request &request, Response &response, Robot  * robot, Commands * server)
{
  for (size_t i = 0; i < request.sub_data_num; i++) {
    uint64_t uid = request.sub_uid_list[i];
    server->create_publisher(uid, request);
  }
  return true;
}

bool DelMsg::answer(Request &request, Response &response, Robot  * robot, Commands * server)
{
  server->stop_publisher(request);
  return true;
}

Commands::Commands(boost::asio::io_context * _io_context, Robot * robot, RoboMaster * rm, short port)
: Server(_io_context, robot, port), robomaster(rm)
{
  register_message<SdkHeartBeat>();
  register_message<SetSdkMode, Commands *>(this);
  register_message<SetRobotMode>();
  register_message<GetRobotMode>();
  register_message<SubNodeReset>();
  register_message<SubscribeAddNode>();
  register_message<VisionDetectEnable, Commands *>(this);
  register_message<ChassisSpeedMode>();
  register_message<AddSubMsg, Commands *>(this);
  register_message<DelMsg, Commands *>(this);
  register_message<GetVersion>();
  register_message<GetProductVersion>();
  register_message<GetSn>();
  register_message<SetSystemLed>();
  register_message<PlaySound, Commands *>(this);
  register_message<PositionMove, Commands *>(this);
  register_message<SetWheelSpeed>();
  register_message<ChassisPwmPercent>();
  register_message<ChassisPwmFreq>();
  register_message<GripperCtrl>();
  register_message<RoboticArmMoveCtrl, Commands *>(this);
  register_message<StreamCtrl, Commands *>(this);
  register_message<VisionDetectStatus>();
  register_message<SetArmorParam>();



  // Currently not used by the robomaster Python library
  // register_message<ChassisSetWorkMode>();
  // register_message<ChassisWheelSpeed>();
  //

  register_subject<VelocitySubject>();
  register_subject<EscSubject>();
  register_subject<AttiInfoSubject>();
  register_subject<ImuSubject>();
  register_subject<PositionSubject>();
  register_subject<SaStatusSubject>();
  register_subject<ChassisModeSubject>();
  register_subject<BatterySubject>();
  register_subject<SbusSubject>();
  register_subject<ArmSubject>();
  register_subject<GripperSubject>();
  register_subject<ServoSubject>();

  spdlog::info("[Commands] Start listening on port {}", port);
  start();
}

void Commands::set_enable_sdk(bool value) {
  if(value) {
    armor_hit_event = std::make_shared<ArmorHitEvent>(this, robot);
  }
  else {
    armor_hit_event = nullptr;
    vision_event = nullptr;
  }
}

void Commands::create_publisher(uint64_t uid, AddSubMsg::Request &request)
{
  uint16_t key = key_from(request.node_id, request.msg_id);
  if(!subjects.count(uid))
  {
    spdlog::warn("Unknown subject uid {}", uid);
    return;
  }
  auto subject = subjects[uid]();
  auto topic= std::make_shared<Topic>(this, robot, request, subject);
  publishers[key] = topic;
  topic->start();
}

void Commands::stop_publisher(DelMsg::Request &request)
{
  uint16_t key = key_from(request.node_id, request.msg_id);
  publishers.erase(key);
}


void Commands::do_step(float time_step)
{
  // spdlog::debug("[Commands] do_step");
  for (auto const& [key, pub] : publishers)
  {
    // spdlog::debug("[Pub] do_step");
    pub->do_step(time_step);
  }
  if(vision_event)
    vision_event->do_step(time_step);
  if(armor_hit_event)
    armor_hit_event->do_step(time_step);
}

VideoStreamer * Commands::get_video_streamer() {
  return robomaster->get_video_streamer();
}

void Commands::set_vision_request(uint8_t sender, uint8_t request, uint16_t mask) {
  vision_event = std::make_shared<VisionEvent>(this, robot, sender, request, mask);
}
