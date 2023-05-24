#include <cstdint>

#include <boost/asio.hpp>

#include "spdlog/spdlog.h"

#include "spdlog/fmt/bin_to_hex.h"
#include "spdlog/fmt/ostr.h"

#include "command.hpp"
#include "command_messages.hpp"
#include "command_subjects.hpp"
#include "event.hpp"
#include "robomaster.hpp"
#include "subscriber_messages.hpp"
#include "utils.hpp"

#define MAX_HEARTBEAT_DELAY 3.0

using boost::asio::ip::udp;

bool AddSubMsg::answer(const Request &request, Response &response, Robot *robot, Commands *server) {
  for (size_t i = 0; i < request.sub_data_num; i++) {
    uint64_t uid = request.sub_uid_list[i];
    server->create_publisher(uid, request);
  }
  return true;
}

bool DelMsg::answer(const Request &request, Response &response, Robot *robot, Commands *server) {
  server->stop_publisher(request);
  return true;
}

Commands::Commands(boost::asio::io_context *_io_context, Robot *robot, RoboMaster *rm,
                   std::string ip, unsigned short port, bool enable_armor_hits, bool enable_ir_hits)
    : Server(_io_context, robot, ip, port)
    , robomaster(rm)
    , enable_armor_hits(enable_armor_hits)
    , enable_ir_hits(enable_ir_hits)
    , _time(0.0)
    , connected(false) {
  register_message<SdkHeartBeat, Commands *>(this);
  register_message<SetSdkMode, Commands *>(this);
  register_message<SetRobotMode>();
  register_message<GetRobotMode>();
  register_message<SubNodeReset, Commands *>(this);
  register_message<SubscribeAddNode, Commands *>(this);
  register_message<VisionDetectEnable, Commands *>(this);
  register_message<ChassisSpeedMode>();
  register_message<AddSubMsg, Commands *>(this);
  register_message<DelMsg, Commands *>(this);
  register_message<GetVersionRM>();
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
  register_message<VisionSetColor>();
  register_message<SensorGetData>();
  register_message<ChassisSerialSet>();
  register_message<ChassisSerialMsgSend>();
  register_message<ServoGetAngle>();
  register_message<ServoModeSet>();
  register_message<ServoControl>();
  register_message<ServoCtrlSet, Commands *>(this);
  register_message<GimbalSetWorkMode>();
  register_message<GimbalCtrlSpeed>();
  register_message<GimbalCtrl>();
  register_message<GimbalRotate, Commands *>(this);
  register_message<GimbalRecenter, Commands *>(this);
  register_message<BlasterFire>();
  register_message<BlasterSetLed>();
  // Currently not used by the robomaster Python library
  register_message<ChassisSetWorkMode>();
  register_message<RoboticArmGetPostion>();
  register_message<GetZoom>();
  register_message<TakePhoto>();
  register_message<SetWhiteBalance>();
  register_message<SetZoom>();

  // register_message<ChassisWheelSpeed>();

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
  register_subject<TofSubject>();
  register_subject<GimbalPosSubject>();
  register_subject<AdapterSubject>();
  spdlog::info("[Commands] Start listening on {}", STREAM(local_endpoint()));
  start();
}

Commands::~Commands() {}

void Commands::add_subscriber_node(uint8_t node_id) {
  spdlog::info("[Commands] add subscriber {}", node_id);
  if (enable_armor_hits)
    armor_hit_event = std::make_unique<ArmorHitEvent>(this, robot, node_id);
  if (enable_ir_hits)
    ir_hit_event = std::make_unique<IRHitEvent>(this, robot, node_id);
  // Disabled
  // uart_event = std::make_unique<UARTEvent>(this, robot);
  last_heartbeat = _time;
  connected = true;
}

void Commands::got_heartbeat() {
  last_heartbeat = _time;
  spdlog::debug("[Commands] got hearbeat at {}", last_heartbeat);
}

void Commands::reset_subscriber_node(uint8_t node_id) {
  spdlog::info("[Commands] reset subscriber {}", node_id);
  unconnect();
}

void Commands::set_enable_sdk(bool value) {
  if (value) {
    spdlog::info("[Commands] enabled the SDK");
  } else {
    spdlog::info("[Commands] disabled the SDK");
    armor_hit_event = nullptr;
    ir_hit_event = nullptr;
    vision_event = nullptr;
    armor_hit_event = nullptr;
    vision_event = nullptr;
    publishers.clear();
  }
}

void Commands::create_publisher(uint64_t uid, const AddSubMsg::Request &request) {
  uint16_t key = key_from(request.node_id, request.msg_id);
  if (!subjects.count(uid)) {
    spdlog::warn("Unknown subject uid {}", uid);
    return;
  }
  // auto subject = subjects[uid]();
  // publishers[key] = topic;
  publishers.emplace(key, std::make_unique<Topic>(this, robot, request, subjects[uid]()));
  publishers[key]->start();
}

void Commands::stop_publisher(const DelMsg::Request &request) {
  uint16_t key = key_from(request.node_id, request.msg_id);
  publishers.erase(key);
}

void Commands::do_step(float time_step) {
  // spdlog::debug("[Commands] do_step");
  for (auto const &[key, pub] : publishers) {
    // spdlog::debug("[Pub] do_step");
    pub->do_step(time_step);
  }
  if (vision_event)
    vision_event->do_step(time_step);
  if (armor_hit_event)
    armor_hit_event->do_step(time_step);
  if (ir_hit_event)
    ir_hit_event->do_step(time_step);
  if (uart_event)
    uart_event->do_step(time_step);
  // check the hearbeat
  _time += time_step;
  if (connected && (_time - last_heartbeat > MAX_HEARTBEAT_DELAY)) {
    spdlog::warn("[Commands] lost connection at {:.3f}, last heart beat received at {:.3f}", _time,
                 last_heartbeat);
    unconnect();
  }
}

VideoStreamer *Commands::get_video_streamer() { return robomaster->get_video_streamer(); }

void Commands::set_vision_request(uint8_t sender, uint8_t request, uint16_t mask) {
  vision_event = std::make_unique<VisionEvent>(this, robot, sender, request, mask);
}

void Commands::unconnect() {
  spdlog::debug("[Commands] unconnect");
  robot->stop_streaming();
  get_video_streamer()->stop();
  armor_hit_event = nullptr;
  ir_hit_event = nullptr;
  vision_event = nullptr;
  publishers.clear();
  connected = false;
}
