
#include <cstdlib>
#include <iostream>
#include <functional>
#include <optional>
#include <cstdint>

#include <boost/asio.hpp>
// #include <boost/ref.hpp>

#include "spdlog/spdlog.h"
#include "spdlog/fmt/bin_to_hex.h"
#include "spdlog/fmt/ostr.h"

#include "server.hpp"
#include "messages.hpp"
#include "command_messages.hpp"
#include "command.hpp"
#include "command_topics.hpp"

using boost::asio::ip::udp;

Commands::Commands(boost::asio::io_context& _io_context, short port)
: Server(_io_context, port),
topics()
{

  add_route<SdkHeartBeat>([](std::shared_ptr<SdkHeartBeat::Request> request) -> std::optional<SdkHeartBeat::Response>{
    SdkHeartBeat::Response response(request);
    return response;
  });

  add_route<SetSdkMode>([](std::shared_ptr<SetSdkMode::Request> request) -> std::optional<SetSdkMode::Response>{
    SetSdkMode::Response response(request);
    return response;
  });

  add_route<SetRobotMode>([](std::shared_ptr<SetRobotMode::Request> request) -> std::optional<SetRobotMode::Response>{
    SetRobotMode::Response response(request);
    return response;
  });

  add_route<GetRobotMode>([](std::shared_ptr<GetRobotMode::Request> request) -> std::optional<GetRobotMode::Response>{
    GetRobotMode::Response response(request);
    response.mode = 1;
    return response;
  });

  add_route<SubNodeReset>([](std::shared_ptr<SubNodeReset::Request> request) -> std::optional<SubNodeReset::Response>{
    SubNodeReset::Response response(request);
    return response;
  });

  add_route<SubscribeAddNode>([](std::shared_ptr<SubscribeAddNode::Request> request) -> std::optional<SubscribeAddNode::Response>{
    SubscribeAddNode::Response response(request);
    response.pub_node_id = 0;
    return response;
  });

  add_route<VisionDetectEnable>([](std::shared_ptr<VisionDetectEnable::Request> request) -> std::optional<VisionDetectEnable::Response>{
    // std::cout << "Need ack: " << int(request->need_ack()) << std::endl;
    VisionDetectEnable::Response response(request);
    return response;
  });

  add_route<ChassisSpeedMode>([](std::shared_ptr<ChassisSpeedMode::Request> request) -> std::optional<ChassisSpeedMode::Response>{
    // std::cout << "Need ack: " << int(request->need_ack()) << std::endl;
    // if(request->need_ack())
    // {
    //   ChassisSpeedMode::Response response(request);
    //   return response;
    // }
    return {};
    // ChassisSpeedMode::Response response(request);
    // response.is_ack = 1;
    // response.need_ack = 0;
    // return response;
  });

  add_route<AddSubMsg>([this](std::shared_ptr<AddSubMsg::Request> request) -> std::optional<AddSubMsg::Response>{
    // std::cout << "Need ack: " << int(request->need_ack()) << std::endl;
    AddSubMsg::Response response(request);
    response.pub_node_id = 0;

    //Start topics;
    for (size_t i = 0; i < request->sub_data_num; i++) {
      uint64_t uid = request->sub_uid_list[i];
      create_publisher_with(uid, request);
    }
    return response;
  });

  add_route<DelMsg>([this](std::shared_ptr<DelMsg::Request> request) -> std::optional<DelMsg::Response>{
    // std::cout << "Need ack: " << int(request->need_ack()) << std::endl;
    DelMsg::Response response(request);
    //Stop topic;
    stop_publisher(request);
    return response;
  });

  add_route<GetVersion>([](std::shared_ptr<GetVersion::Request> request) -> std::optional<GetVersion::Response>{
    GetVersion::Response response(request);
    response.aa = 0;
    response.bb = 1;
    response.cc = 2;
    response.dd = 3;
    return response;
  });

  add_route<GetProductVersion>([](std::shared_ptr<GetProductVersion::Request> request) -> std::optional<GetProductVersion::Response>{
    GetProductVersion::Response response(request);
    response.aa = 0;
    response.bb = 1;
    response.cc = 2;
    return response;
  });

  // add_route<GetSn>([](std::shared_ptr<GetSn::Request> request) -> std::optional<GetSn::Response>{
  //   GetSn::Response response(request);
  //   response.serial_number = std::string("serial_1234");
  //   return response;
  // });

  add_route<ChassisWheelSpeed>([](std::shared_ptr<ChassisWheelSpeed::Request> request) -> std::optional<ChassisWheelSpeed::Response>{
    ChassisWheelSpeed::Response response(request);
    return response;
  });

  add_route<SetSystemLed>([](std::shared_ptr<SetSystemLed::Request> request) -> std::optional<SetSystemLed::Response>{
    SetSystemLed::Response response(request);
    return response;
  });

  add_route<PlaySound>([](std::shared_ptr<PlaySound::Request> request) -> std::optional<PlaySound::Response>{
    PlaySound::Response response(request);
    response.accept = 1;
    return response;
  });

  add_route<PositionMove>([](std::shared_ptr<PositionMove::Request> request) -> std::optional<PositionMove::Response>{
    PositionMove::Response response(request);
    response.accept = 1;
    return response;
  });

  add_route<PositionPush>([](std::shared_ptr<PositionPush::Request> request) -> std::optional<PositionPush::Response>{
    PositionPush::Response response(request);
    return response;
  });

  add_route<SetWheelSpeed>([](std::shared_ptr<SetWheelSpeed::Request> request) -> std::optional<SetWheelSpeed::Response>{
    SetWheelSpeed::Response response(request);
    return response;
  });

  add_route<ChassisPwmPercent>([](std::shared_ptr<ChassisPwmPercent::Request> request) -> std::optional<ChassisPwmPercent::Response>{
    ChassisPwmPercent::Response response(request);
    return response;
  });

  add_route<ChassisPwmFreq>([](std::shared_ptr<ChassisPwmFreq::Request> request) -> std::optional<ChassisPwmFreq::Response>{
    ChassisPwmFreq::Response response(request);
    return response;
  });

  add_route<GripperCtrl>([](std::shared_ptr<GripperCtrl::Request> request) -> std::optional<GripperCtrl::Response>{
    GripperCtrl::Response response(request);
    return response;
  });

  add_route<ChassisSetWorkMode>([](std::shared_ptr<ChassisSetWorkMode::Request> request) -> std::optional<ChassisSetWorkMode::Response>{
    ChassisSetWorkMode::Response response(request);
    return response;
  });

  add_route<RoboticArmMoveCtrl>([](std::shared_ptr<RoboticArmMoveCtrl::Request> request) -> std::optional<RoboticArmMoveCtrl::Response>{
    RoboticArmMoveCtrl::Response response(request);
    response.accept = true;
    return response;
  });

  add_route<RoboticArmMovePush>([](std::shared_ptr<RoboticArmMovePush::Request> request) -> std::optional<RoboticArmMovePush::Response>{
    RoboticArmMovePush::Response response(request);
    // TODO(jerome): actions
    return response;
  });

  add_route<StreamCtrl>([](std::shared_ptr<StreamCtrl::Request> request) -> std::optional<StreamCtrl::Response>{
    StreamCtrl::Response response(request);
    return response;
  });


  spdlog::info("[Commands] Start listening on port {}", port);
  start();
}


void Commands::create_publisher_with(uint64_t uid, std::shared_ptr<AddSubMsg::Request> request)
{
  spdlog::info("[Commands] try to create publisher with uid {}", uid);
  if(uid == 0x0002000949a4009c)
  {
    create_publisher<VelocitySubject>(request);
    return;
  }
  if(uid == 0x00020009c14cb7c5)
  {
    create_publisher<EscSubject>(request);
    return;
  }
  if(uid == 0x000200096b986306)
  {
    create_publisher<AttiInfoSubject>(request);
    return;
  }
  if(uid == 0x00020009a7985b8d)
  {
    create_publisher<ImuSubject>(request);
    return;
  }
  if(uid == 0x00020009eeb7cece)
  {
    create_publisher<PositionSubject>(request);
    return;
  }
  if(uid == 0x000200094a2c6d55)
  {
    create_publisher<SaStatusSubject>(request);
    return;
  }
  if(uid == 0x000200094fcb1146)
  {
    create_publisher<ChassisModeSubject>(request);
    return;
  }
  if(uid == 0x0002000988223568)
  {
    create_publisher<SbusSubject>(request);
    return;
  }
  if(uid == 0x000200096862229f)
  {
    create_publisher<BatterySubject>(request);
    return;
  }
  if(uid == 0x00020009124d156a)
  {
    create_publisher<GripperSubject>(request);
    return;
  }
  if(uid == 0x0002000926abd64d)
  {
    create_publisher<ArmSubject>(request);
    return;
  }
  spdlog::warn("unknown uid!");
}

template<typename T>
std::shared_ptr<T> Commands::create_publisher(std::shared_ptr<AddSubMsg::Request> request)
{
  auto topic= std::make_shared<T>(io_context, this, request);
  uint16_t key = key_from(request->node_id, request->msg_id);
  publishers[key] = topic;
  topic->start();
  return topic;
}

void Commands::stop_publisher(std::shared_ptr<DelMsg::Request> request)
{
  uint16_t key = key_from(request->node_id, request->msg_id);
  publishers.erase(key);
}
