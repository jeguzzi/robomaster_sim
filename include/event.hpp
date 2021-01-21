#ifndef EVENT_H
#define EVENT_H

#include <memory>
#include "robot.hpp"
#include "protocol.hpp"
#include "command.hpp"

template <typename B>
struct Event
{
  Commands * cmd;
  Robot * robot;
  uint8_t sender;
  uint8_t receiver;

  Event(Commands * cmd, Robot * robot, uint8_t sender, uint8_t receiver)
  : cmd(cmd), robot(robot), sender(sender), receiver(receiver) {};

  void do_step(float time_step) {
    for(auto & msg : update_msg()) {
      auto data = msg->encode_msg(B::set, B::cmd);
      spdlog::debug("Push Event Msg {} bytes: {:n}", data.size(), spdlog::to_hex(data));
      cmd->send(data);
    }
  }
  virtual std::vector<std::shared_ptr<typename B::Response>> update_msg() = 0;
  virtual ~Event(){};
};


struct VisionDetectInfo : Proto<0xa, 0xa4>
{
  struct Response : ResponseT{

    Response(uint8_t sender, uint8_t receiver, uint16_t type)
    : ResponseT(sender, receiver), type(type)
    {
      is_ack = 0;
      need_ack = 0;
    }

    detected_items_t items;
    uint8_t type;
    uint8_t status;
    uint16_t errcode;

    std::vector<uint8_t> encode()
    {
      std::vector<uint8_t> buffer(20 * items.size() + 9, 0);
      buffer[0] = type;
      buffer[1] = status;
      write<uint16_t>(buffer, 6, errcode);
      buffer[8] = items.size();
      size_t i = 9;
      for(auto & object : items) {
        object->encode(buffer, i);
        i += 20;
      }
      return buffer;
    }

  };

};


struct VisionEvent : Event<VisionDetectInfo> {

  VisionEvent(Commands * cmd, Robot * robot, uint8_t sender, uint8_t receiver, uint8_t type) :
    Event(cmd, robot, sender, receiver), type(type){};

  std::vector<std::shared_ptr<VisionDetectInfo::Response>> update_msg() {
    auto objects = robot->get_detected_objects();
    std::vector<std::shared_ptr<VisionDetectInfo::Response>> msgs;
    for (auto const& [key, objects] : *objects) {
      if(type & (1 << key)) {
        auto msg = std::make_shared<VisionDetectInfo::Response>(sender, receiver, key);
        msg->items = objects;
        msg->status = 0;
        msgs.push_back(msg);
      }
    }
    return msgs;
  }

  uint8_t type;

};


struct ArmorHitEventMsg : Proto<0x3f, 0x02>
{
  struct Response : ResponseT{

    Response(uint8_t sender, uint8_t receiver, uint8_t type, uint8_t index,
             uint16_t mic_value, uint16_t mic_len)
    : ResponseT(sender, receiver), type(type), index(index), mic_value(mic_value), mic_len(mic_len)
    {
    }

    detected_items_t items;
    uint8_t type;
    uint8_t index;
    uint16_t mic_value;
    uint16_t mic_len;

    std::vector<uint8_t> encode()
    {
      std::vector<uint8_t> buffer(5, 0);
      buffer[0] = (index << 4) | type;
      write<uint16_t>(buffer, 1, mic_value);
      write<uint16_t>(buffer, 3, mic_len);
      return buffer;
    }
  };

};

struct ArmorHitEvent : Event<ArmorHitEventMsg> {

  ArmorHitEvent(Commands * cmd, Robot * robot, uint8_t sender=0xc9, uint8_t receiver=0x38) :
    Event(cmd, robot, sender, receiver) {};

  std::vector<std::shared_ptr<ArmorHitEventMsg::Response>> update_msg() {
    auto hits = robot->get_hit_events();
    std::vector<std::shared_ptr<ArmorHitEventMsg::Response>> msgs;
    for (auto const& hit : hits) {
      auto msg = std::make_shared<ArmorHitEventMsg::Response>(sender, receiver, hit.type, hit.index, 0, 0);
      msgs.push_back(msg);
    }
    return msgs;
  }


  uint8_t type;

};

#endif /* end of include guard: EVENT_H */
