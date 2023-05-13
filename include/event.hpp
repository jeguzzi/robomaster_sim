#ifndef INCLUDE_EVENT_HPP_
#define INCLUDE_EVENT_HPP_

#include <algorithm>
#include <memory>
#include <vector>

#include "command.hpp"
#include "protocol.hpp"
#include "robot/robot.hpp"

template <typename B> struct Event {
  Commands *cmd;
  Robot *robot;
  uint8_t sender;
  uint8_t receiver;

  Event(Commands *cmd, Robot *robot, uint8_t sender, uint8_t receiver)
      : cmd(cmd)
      , robot(robot)
      , sender(sender)
      , receiver(receiver) {}

  void do_step(float time_step) {
    for (auto &msg : update_msg()) {
      auto data = msg.encode_msg(B::set, B::cmd);
      spdlog::debug("Push Event Msg {} bytes: {:n}", data.size(), spdlog::to_hex(data));
      cmd->send(data);
    }
  }
  virtual std::vector<typename B::Response> update_msg() = 0;
  virtual ~Event() {}
};

void encode(std::vector<uint8_t> &buffer, size_t location, const BoundingBox &object) {
  write<float>(buffer, location + 0, object.x);
  write<float>(buffer, location + 4, object.y);
  write<float>(buffer, location + 8, object.width);
  write<float>(buffer, location + 12, object.height);
}

void encode(std::vector<uint8_t> &buffer, size_t location, const DetectedObjects::Person &object) {
  encode(buffer, location, object.bounding_box);
}

void encode(std::vector<uint8_t> &buffer, size_t location, const DetectedObjects::Gesture &object) {
  encode(buffer, location, object.bounding_box);
  write<uint32_t>(buffer, location + 16, object.id);
}

void encode(std::vector<uint8_t> &buffer, size_t location, const DetectedObjects::Line &object) {
  write<float>(buffer, location + 0, object.x);
  write<float>(buffer, location + 4, object.y);
  write<float>(buffer, location + 8, object.angle);
  write<float>(buffer, location + 12, object.curvature);
  write<uint32_t>(buffer, location + 16, object.info);
}

void encode(std::vector<uint8_t> &buffer, size_t location, const DetectedObjects::Marker &object) {
  encode(buffer, location, object.bounding_box);
  write<uint16_t>(buffer, location + 16, object.id);
  write<uint16_t>(buffer, location + 18, object.distance);
}

void encode(std::vector<uint8_t> &buffer, size_t location, const DetectedObjects::Robot &object) {
  encode(buffer, location, object.bounding_box);
}

struct VisionDetectInfo : Proto<0xa, 0xa4> {
  struct Response : ResponseT {
    Response(uint8_t sender, uint8_t receiver, uint16_t type, uint8_t number)
        : ResponseT(sender, receiver)
        , type(type)
        , number(number)
        , status(0)
        , errcode(0)
        , buffer(20 * number + 9) {}

    uint8_t type;
    uint8_t number;
    uint8_t status;
    uint16_t errcode;
    std::vector<uint8_t> buffer;

    std::vector<uint8_t> encode() {
      buffer[0] = type;
      buffer[1] = status;
      write<uint16_t>(buffer, 6, errcode);
      buffer[8] = number;
      // buffer is encoded externally to get around polymorphic items
      return buffer;
    }
  };
};

struct VisionEvent : Event<VisionDetectInfo> {
  VisionEvent(Commands *cmd, Robot *robot, uint8_t sender, uint8_t receiver, uint8_t type)
      : Event(cmd, robot, sender, receiver)
      , type(type) {}

  template <typename T>
  void add_message(std::vector<VisionDetectInfo::Response> &msgs, const DetectedObjects &objects) {
    if (type & (1 << T::type)) {
      if (!robot->vision.is_enabled<T>()) return;
      auto items = objects.get<T>();
      auto size = items.size();
      if (true || size) {
        VisionDetectInfo::Response msg(sender, receiver, T::type, size);
        size_t location = 9;
        for (auto &item : items) {
          encode(msg.buffer, location, item);
          location += 20;
        }
        msgs.push_back(msg);
      }
    }
  }

  std::vector<VisionDetectInfo::Response> update_msg() {
    auto objects = robot->vision.get_detected_objects();
    std::vector<VisionDetectInfo::Response> msgs;
    add_message<DetectedObjects::Person>(msgs, objects);
    add_message<DetectedObjects::Gesture>(msgs, objects);
    add_message<DetectedObjects::Line>(msgs, objects);
    add_message<DetectedObjects::Marker>(msgs, objects);
    add_message<DetectedObjects::Robot>(msgs, objects);
    return msgs;
  }

  uint8_t type;
};

struct ArmorHitEventMsg : Proto<0x3f, 0x02> {
  struct Response : ResponseT {
    Response(uint8_t sender, uint8_t receiver, uint8_t type, uint8_t index, uint16_t mic_value,
             uint16_t mic_len)
        : ResponseT(sender, receiver)
        , type(type)
        , index(index)
        , mic_value(mic_value)
        , mic_len(mic_len) {}

    uint8_t type;
    uint8_t index;
    uint16_t mic_value;
    uint16_t mic_len;

    std::vector<uint8_t> encode() {
      std::vector<uint8_t> buffer(5, 0);
      buffer[0] = (index << 4) | type;
      write<uint16_t>(buffer, 1, mic_value);
      write<uint16_t>(buffer, 3, mic_len);
      return buffer;
    }
  };
};

struct ArmorHitEvent : Event<ArmorHitEventMsg> {
  ArmorHitEvent(Commands *cmd, Robot *robot, uint8_t sender = 0xc9, uint8_t receiver = 0x38)
      : Event(cmd, robot, sender, receiver) {}

  std::vector<ArmorHitEventMsg::Response> update_msg() {
    auto hits = robot->armor.get_hit_events();
    std::vector<ArmorHitEventMsg::Response> msgs;
    for (auto const &hit : hits) {
      msgs.emplace_back(sender, receiver, hit.type, hit.index, 0, 0);
    }
    return msgs;
  }

  uint8_t type;
};

struct IRHitEventMsg : Proto<0x3f, 0x10> {
  struct Response : ResponseT {
    uint8_t skill_id;
    uint8_t role_id;
    uint8_t recv_dev;
    uint8_t recv_ir_pin;

    Response(uint8_t sender, uint8_t receiver, uint8_t skill_id = 0, uint8_t role_id = 0,
             uint16_t recv_dev = 0, uint16_t recv_ir_pin = 0)
        : ResponseT(sender, receiver)
        , skill_id(skill_id)
        , role_id(role_id)
        , recv_dev(recv_dev)
        , recv_ir_pin(recv_ir_pin) {}

    std::vector<uint8_t> encode() {
      uint8_t b = (role_id << 4) | skill_id;
      return {b, recv_dev, recv_ir_pin};
    }
  };
};

struct IRHitEvent : Event<IRHitEventMsg> {
  IRHitEvent(Commands *cmd, Robot *robot, uint8_t sender = 0xc9, uint8_t receiver = 0x38)
      : Event(cmd, robot, sender, receiver) {}

  std::vector<IRHitEventMsg::Response> update_msg() {
    auto hits = robot->armor.get_ir_events();
    std::vector<IRHitEventMsg::Response> msgs;
    for (auto const &hit : hits) {
      msgs.emplace_back(sender, receiver, hit.skill_id, hit.role_id, hit.recv_dev, hit.recv_ir_pin);
    }
    return msgs;
  }

  uint8_t type;
};

struct UARTMessage : Proto<0x3f, 0xc1> {
  struct Response : ResponseT {
    Response(uint8_t sender, uint8_t receiver, uint16_t length, const uint8_t *buffer)
        : ResponseT(sender, receiver)
        , length(length)
        , buffer(buffer, buffer + length) {}

    uint16_t length;
    std::vector<uint8_t> buffer;

    std::vector<uint8_t> encode() {
      buffer.insert(buffer.begin(), length & 0xFF);
      buffer.insert(buffer.begin(), length >> 8);
      buffer.insert(buffer.begin(), 1);
      return buffer;
    }
  };
};

struct UARTEvent : Event<UARTMessage> {
  UARTEvent(Commands *cmd, Robot *robot, uint8_t sender = 0xc9, uint8_t receiver = 0x66)
      : Event(cmd, robot, sender, receiver) {}

  std::vector<UARTMessage::Response> update_msg() {
    std::vector<UARTMessage::Response> msgs;
    msgs.emplace_back(sender, receiver, 5, (const uint8_t *)"Hello");
    return msgs;
  }
};

#endif  // INCLUDE_EVENT_HPP_
