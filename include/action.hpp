#ifndef ACTION
#define ACTION

#include <iostream>
#include <cstdint>
#include <algorithm>

#include "protocol.hpp"
#include "utils.hpp"

#include <spdlog/spdlog.h>
#include "spdlog/fmt/ostr.h"

class Robot;

struct Action
{
  enum State : uint8_t {
    running = 0,
    succeed = 1,
    failed = 2,
    started = 3,
    undefined = 4,
  };

  Action(uint8_t _id, float _frequency)
  : id(_id), frequency(_frequency), state(State::undefined), deadline(0) {
  }


  uint8_t accept_code() {
    if(state == State::running || state == State::started) return 0;
    if(state == State::succeed) return 2;
    return 1;
  }

  virtual std::vector<uint8_t> do_step(float time_step) = 0;

  virtual ~Action() {};

  uint8_t id;
  float frequency;
  State state;
  float deadline;
};

template <typename T>
struct TAction : Action {
  TAction(uint8_t _id, float _frequency, std::shared_ptr<typename T::Response> push)
  : Action(_id, _frequency), push_msg(push) {
    push->action_id = id;
}

  ~TAction() {} ;

  virtual void update() = 0;

  std::vector<uint8_t> do_step(float time_step) {
    deadline -= time_step;
    if(deadline <= 0) {
      deadline += 1.0f / frequency;
      update();
      return push_msg->encode_msg(T::set, T::cmd);
    }
    return {};
  }

  std::shared_ptr<typename T::Response> push_msg;
};

float time_to_goal(Pose2D & goal_pose, float linear_speed, float angular_speed);

struct PositionPush : Proto<0x3f, 0x2a>
{
  struct Response : ResponseT{
    uint8_t action_id;
    uint8_t percent;
    uint8_t action_state;
    int16_t pos_x;
    int16_t pos_y;
    int16_t pos_z;

    std::vector<uint8_t> encode()
    {
      std::vector<uint8_t> buffer(9, 0);
      buffer[0] = action_id;
      buffer[1] = percent;
      buffer[2] = action_state;
      write<int16_t>(buffer, 3, pos_x);
      write<int16_t>(buffer, 5, pos_y);
      write<int16_t>(buffer, 7, pos_z);
      return buffer;
    };
    using ResponseT::ResponseT;
  };

};

struct MoveAction : TAction<PositionPush>
{
  MoveAction(uint8_t _id, float _frequency, std::shared_ptr<PositionPush::Response> push,
    Pose2D goal_pose, float _linear_speed, float _angular_speed)
  : TAction<PositionPush>(_id, _frequency, push), goal(goal_pose), linear_speed(_linear_speed), angular_speed(_angular_speed) {
    predicted_duration = time_to_goal(goal_pose, linear_speed, angular_speed);
  }
  void update() {
    push_msg->seq_id++;
    if (state == Action::State::succeed) {
      push_msg->percent = 100;
    }
    else {
      push_msg->percent = std::max(0, (int)(100 - round(100.0f * remaining_duration / predicted_duration)));
    }
    push_msg->action_state = state;
    std::cout << "current: " << current << std::endl;
    // TODO(jerome): check if coherent with real robot
    push_msg->pos_x = (int16_t) round(100 * current.x);
    push_msg->pos_y = -(int16_t) round(100 * current.y);
    push_msg->pos_z = (int16_t) round(10 * rad2deg(current.theta));
    // push_msg->pos_y = ...
    // push_msg->pos_z = ...
  }
  Pose2D goal;
  Pose2D goal_odom;
  Pose2D current;
  float linear_speed;
  float angular_speed;
  float predicted_duration;
  float remaining_duration;
};



#endif /* end of include guard: ACTION */
