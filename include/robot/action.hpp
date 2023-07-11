#ifndef INCLUDE_ROBOT_ACTION_HPP_
#define INCLUDE_ROBOT_ACTION_HPP_

#include <algorithm>

#include "../utils.hpp"
#include "gimbal.hpp"

class Robot;

using Callback = std::function<void(float)>;

struct Action {
  enum State : uint8_t {
    running = 0,
    succeed = 1,
    failed = 2,
    started = 3,
    undefined = 4,
    rejected = 5,
  };
  explicit Action(Robot *robot)
      : robot(robot)
      , state(State::undefined)
      , callback([](float) {}) {}
  virtual void do_step(float time_step) = 0;
  void do_step_cb(float time_step) {
    do_step(time_step);
    callback(time_step);
  }
  virtual ~Action() {}
  bool done() { return state == Action::State::failed || state == Action::State::succeed; }
  void set_callback(Callback value) { callback = value; }

  Robot *robot;
  State state;
  float predicted_duration;
  float remaining_duration;
  Callback callback;
};

#if FMT_VERSION >= 90000 
template <> struct fmt::formatter<Action::State>: formatter<std::string_view> {
  auto format(Action::State value, format_context& ctx) const {
  std::string_view name = "";
  switch (value) {
    case Action::State::running: name = "running"; break;
    case Action::State::succeed: name = "succeed"; break;
    case Action::State::failed: name = "failed"; break;
    case Action::State::started: name = "started"; break;
    case Action::State::undefined: name = "undefined"; break;
    case Action::State::rejected: name = "rejected"; break;
  }
  return formatter<string_view>::format(name, ctx);    
  }
};
#endif

inline float time_to_goal(const Pose2D &goal_pose, float linear_speed, float angular_speed) {
  return std::max(std::abs(normalize(goal_pose.theta)) / angular_speed,
                  goal_pose.distance() / linear_speed);
}

struct MoveAction : Action {
  MoveAction(Robot *robot, Pose2D goal_pose, float _linear_speed, float _angular_speed)
      : Action(robot)
      , goal(goal_pose)
      , linear_speed(_linear_speed)
      , angular_speed(_angular_speed) {
    predicted_duration = time_to_goal(goal_pose, linear_speed, angular_speed);
  }
  virtual void do_step(float time_step);
  Pose2D goal;
  Pose2D goal_odom;
  Pose2D current;
  float linear_speed;
  float angular_speed;
};

struct MoveArmAction : Action {
  MoveArmAction(Robot *robot, float x, float z, bool _absolute)
      : Action(robot)
      , goal_position({x, 0, z})
      , absolute(_absolute) {}

  virtual void do_step(float time_step);
  Vector3 goal_position;
  bool absolute;
  // Vector3 current_position;
};

struct PlaySoundAction : Action {
  static constexpr float duration = 3.0;

  PlaySoundAction(Robot *robot, uint32_t _sound_id, uint8_t _play_times)
      : Action(robot)
      , sound_id(_sound_id)
      , play_times(_play_times) {
    predicted_duration = duration;
    remaining_duration = 0.0;
  }
  virtual void do_step(float time_step);
  uint32_t sound_id;
  uint8_t play_times;
};

struct MoveServoAction : Action {
  static constexpr float MAX_DURATION = 5.0;
  MoveServoAction(Robot *robot, size_t servo_id, float target_angle)
      : Action(robot)
      , servo_id(servo_id)
      , target_angle(target_angle)
      , time_left(MAX_DURATION) {}
  virtual void do_step(float time_step);
  size_t servo_id;
  float target_angle;
  float current_angle;
  float time_left;
};

struct MoveGimbalAction : Action {
  MoveGimbalAction(Robot *robot, float target_yaw, float target_pitch, float yaw_speed,
                   float pitch_speed, Gimbal::Frame yaw_frame, Gimbal::Frame pitch_frame)
      : Action(robot)
      , target({.yaw = target_yaw, .pitch = target_pitch})
      , speed({.yaw = yaw_speed, .pitch = pitch_speed})
      , frame({.yaw = yaw_frame, .pitch = pitch_frame}) {}

  virtual void do_step(float time_step);
  GimbalValues<float> target;
  GimbalValues<float> current;
  GimbalValues<float> speed;
  GimbalValues<Gimbal::Frame> frame;
};

#endif  // INCLUDE_ROBOT_ACTION_HPP_ */
