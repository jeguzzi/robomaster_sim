#ifndef INCLUDE_ACTION_HPP_
#define INCLUDE_ACTION_HPP_

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <memory>
#include <utility>
#include <vector>

#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"

#include "protocol.hpp"
#include "robot/robot.hpp"
#include "utils.hpp"

class Commands;

template <typename T> struct ActionSDK {
  void publish(float time_step) {
    deadline -= time_step;
    if (action->done() || deadline <= 0) {
      deadline += 1.0f / frequency;
      update_msg();
      cmd->send(push_msg->encode_msg(T::set, T::cmd));
    }
  }

  ActionSDK(Commands *_cmd, uint8_t _id, float _frequency,
            std::unique_ptr<typename T::Response> _push, Action *_action)
      : cmd(_cmd)
      , id(_id)
      , frequency(_frequency)
      , deadline(0)
      , push_msg(std::move(_push))
      , action(_action) {
    push_msg->action_id = id;
    action->set_callback(std::bind(&ActionSDK::publish, this, std::placeholders::_1));
  }

  ~ActionSDK() {}

  virtual void update_msg() = 0;

  Commands *cmd;
  uint8_t id;
  float frequency;
  float deadline;
  std::unique_ptr<typename T::Response> push_msg;
  Action *action;
};

struct PositionPush : Proto<0x3f, 0x2a> {
  struct Response : ResponseT {
    uint8_t action_id;
    uint8_t percent;
    uint8_t action_state;
    int16_t pos_x;
    int16_t pos_y;
    int16_t pos_z;

    std::vector<uint8_t> encode() {
      std::vector<uint8_t> buffer(9, 0);
      buffer[0] = action_id;
      buffer[1] = percent;
      buffer[2] = action_state;
      write<int16_t>(buffer, 3, pos_x);
      write<int16_t>(buffer, 5, pos_y);
      write<int16_t>(buffer, 7, pos_z);
      return buffer;
    }
    using ResponseT::ResponseT;
  };
};

struct MoveActionSDK : MoveAction, ActionSDK<PositionPush> {
  MoveActionSDK(Commands *cmd, uint8_t id, float frequency,
                std::unique_ptr<PositionPush::Response> push, Robot *robot, Pose2D goal,
                float linear_speed, float angular_speed)
      : MoveAction(robot, goal, linear_speed, angular_speed)
      , ActionSDK<PositionPush>(cmd, id, frequency, std::move(push), this) {
    action = this;
  }
  void update_msg() {
    push_msg->seq_id++;
    if (state == State::succeed) {
      push_msg->percent = 100;
    } else {
      push_msg->percent = std::max(
          0, static_cast<int>(100 - round(100.0f * remaining_duration / predicted_duration)));
    }
    push_msg->action_state = state;
    // std::cout << "current: " << current << std::endl;
    // TODO(Jerome): check if coherent with real robot
    Pose2D current = robot->chassis.get_pose();
    push_msg->pos_x = (int16_t)round(100 * current.x);
    push_msg->pos_y = -(int16_t)round(100 * current.y);
    push_msg->pos_z = (int16_t)round(10 * rad2deg(current.theta));
    // push_msg->pos_y = ...
    // push_msg->pos_z = ...
  }
};

struct RoboticArmMovePush : Proto<0x3f, 0xb6> {
  struct Response : ResponseT {
    uint8_t action_id;
    uint8_t percent;
    uint8_t action_state;
    int32_t x;
    int32_t y;

    std::vector<uint8_t> encode() {
      std::vector<uint8_t> buffer(11, 0);
      buffer[0] = action_id;
      buffer[1] = percent;
      buffer[2] = action_state;
      write<int32_t>(buffer, 3, x);
      write<int32_t>(buffer, 7, y);
      return buffer;
    }
    using ResponseT::ResponseT;
  };
};

struct MoveArmActionSDK : MoveArmAction, ActionSDK<RoboticArmMovePush> {
  MoveArmActionSDK(Commands *cmd, uint8_t id, float frequency,
                   std::unique_ptr<RoboticArmMovePush::Response> push, Robot *robot, float x,
                   float z, bool absolute)
      : MoveArmAction(robot, x, z, absolute)
      , ActionSDK<RoboticArmMovePush>(cmd, id, frequency, std::move(push), this) {
    action = this;
  }

  void update_msg() {
    push_msg->seq_id++;
    if (state == Action::State::succeed) {
      push_msg->percent = 100;
    } else {
      push_msg->percent = std::max(
          0, static_cast<int>(100 - round(100.0f * remaining_duration / predicted_duration)));
    }
    push_msg->action_state = state;
    // std::cout << "current: " << current_position << std::endl;
    // TODO(Jerome): check if coherent with real robot
    Vector3 current_position = robot->arm.get_position();
    push_msg->x = (int32_t)round(1000 * current_position.x);
    push_msg->y = (int32_t)round(1000 * current_position.z);
  }
};

struct PlaySoundPush : Proto<0x3f, 0xb4> {
  struct Response : ResponseT {
    uint8_t action_id;
    uint8_t percent;
    uint8_t action_state;
    int32_t sound_id;
    uint8_t error_reason;

    std::vector<uint8_t> encode() {
      std::vector<uint8_t> buffer(11, 7);
      buffer[0] = action_id;
      buffer[1] = percent;
      buffer[2] = (error_reason << 2) | action_state;
      write<int32_t>(buffer, 3, sound_id);
      return buffer;
    }
    using ResponseT::ResponseT;
  };
};

struct PlaySoundActionSDK : PlaySoundAction, ActionSDK<PlaySoundPush> {
  PlaySoundActionSDK(Commands *cmd, uint8_t id, float frequency,
                     std::unique_ptr<PlaySoundPush::Response> push, Robot *robot, uint32_t sound_id,
                     uint8_t play_times)
      : PlaySoundAction(robot, sound_id, play_times)
      , ActionSDK<PlaySoundPush>(cmd, id, frequency, std::move(push), this) {
    action = this;
  }

  void update_msg() {
    push_msg->sound_id = sound_id;
    push_msg->percent = std::max(
        0, static_cast<int>(100 - round(100.0f * remaining_duration / predicted_duration)));
    push_msg->action_state = state;
  }
};

struct ServoCtrlPush : Proto<0x3f, 0xb8> {
  struct Response : ResponseT {
    uint8_t action_id;
    uint8_t percent;
    uint8_t action_state;
    int32_t value;

    std::vector<uint8_t> encode() {
      std::vector<uint8_t> buffer(7, 0);
      buffer[0] = action_id;
      buffer[1] = percent;
      buffer[2] = action_state;
      write<int32_t>(buffer, 3, value);
      return buffer;
    }
    using ResponseT::ResponseT;
  };
};

struct MoveServoActionSDK : MoveServoAction, ActionSDK<ServoCtrlPush> {
  MoveServoActionSDK(Commands *cmd, uint8_t id, float frequency,
                     std::unique_ptr<ServoCtrlPush::Response> push, Robot *robot, size_t servo_id,
                     float angle)
      : MoveServoAction(robot, servo_id, angle)
      , ActionSDK<ServoCtrlPush>(cmd, id, frequency, std::move(push), this) {
    action = this;
  }

  void update_msg() {
    push_msg->value = servo_angle_value(servo_id, current_angle);
    push_msg->percent = std::max(
        0, static_cast<int>(100 - round(100.0f * remaining_duration / predicted_duration)));
    push_msg->action_state = state;
  }
};

struct GimbalActionPush : Proto<0x3f, 0xb1> {
  struct Response : ResponseT {
    uint8_t action_id;
    uint8_t percent;
    uint8_t action_state;
    int16_t yaw;
    int16_t roll;
    int16_t pitch;

    std::vector<uint8_t> encode() {
      std::vector<uint8_t> buffer(9, 0);
      buffer[0] = action_id;
      buffer[1] = percent;
      buffer[2] = action_state;
      write<int16_t>(buffer, 3, yaw);
      write<int16_t>(buffer, 5, roll);
      write<int16_t>(buffer, 7, pitch);
      return buffer;
    }
    using ResponseT::ResponseT;
  };
};

struct GimbalActionSDK : MoveGimbalAction, ActionSDK<GimbalActionPush> {
  GimbalActionSDK(Commands *cmd, uint8_t id, float frequency,
                  std::unique_ptr<GimbalActionPush::Response> push, Robot *robot, float yaw,
                  float pitch, float yaw_speed, float pitch_speed, Gimbal::Frame yaw_frame,
                  Gimbal::Frame pitch_frame)
      : MoveGimbalAction(robot, yaw, pitch, yaw_speed, pitch_speed, yaw_frame, pitch_frame)
      , ActionSDK<GimbalActionPush>(cmd, id, frequency, std::move(push), this) {
    action = this;
  }

  void update_msg() {
    push_msg->yaw = round(rad2deg(10 * current.yaw));
    push_msg->pitch = round(rad2deg(10 * current.pitch));
    push_msg->percent = std::max(
        0, static_cast<int>(100 - round(100.0f * remaining_duration / predicted_duration)));
    push_msg->action_state = state;
  }
};

#endif  // INCLUDE_ACTION_HPP_
