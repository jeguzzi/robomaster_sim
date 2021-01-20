
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

Commands::Commands(boost::asio::io_context * _io_context, Robot * robot, short port)
: Server(_io_context, robot, port)
{
  robot->set_commands(this);
  register_message<SdkHeartBeat>();
  register_message<SetSdkMode>();
  register_message<SetRobotMode>();
  register_message<GetRobotMode>();
  register_message<SubNodeReset>();
  register_message<SubscribeAddNode>();
  register_message<VisionDetectEnable>();
  register_message<ChassisSpeedMode>();
  register_message<AddSubMsg, Commands *>(this);
  register_message<DelMsg, Commands *>(this);
  register_message<GetVersion>();
  register_message<GetProductVersion>();
  register_message<GetSn>();
  register_message<SetSystemLed>();
  register_message<PlaySound>();
  register_message<PositionMove>();
  // register_message<PositionPush>();
  register_message<SetWheelSpeed>();
  register_message<ChassisPwmPercent>();
  register_message<ChassisPwmFreq>();
  register_message<GripperCtrl>();

  register_message<RoboticArmMoveCtrl>();
  // register_message<RoboticArmMovePush>();
  register_message<StreamCtrl>();

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
}
