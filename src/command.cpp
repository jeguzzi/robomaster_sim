
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
#include "command.hpp"

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
    if(request->need_ack())
    {
      ChassisSpeedMode::Response response(request);
      return response;
    }
    return {};
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

  spdlog::info("[Commands] Start listening on port {}", port);
  start();
}


void Commands::create_publisher_with(uint64_t uid, std::shared_ptr<AddSubMsg::Request> request)
{
  if(uid == 0x0002000949a4009c)
  {
    create_publisher<VelocitySubject>(request);
    return;
  }
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
