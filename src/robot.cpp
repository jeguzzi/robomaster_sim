#include <math.h>
#include <string.h>
#include <stdexcept>
#include <algorithm>

#include <iostream>
#include <fstream>
#include <memory>

#include <spdlog/spdlog.h>

#include "robot.hpp"
#include "command.hpp"
#include "discovery.hpp"
#include "utils.hpp"
#include "action.hpp"
#include "streamer.hpp"

static Color breath_led(float _time, Color color, float period_1, float period_2) {
  float f;
  if(_time < period_1){
    f = sin(_time/period_1 * M_PI_2);
  }
  else {
    f = cos((_time - period_1)/period_2 * M_PI_2);
  }
  spdlog::debug("breath {} {}: {} -> {}", period_1, period_2, _time, f);
  return color * (f * f);
}

static Color flash_led(float _time, Color color, float period_1, float period_2) {
  if(_time < period_1) return color;
  return {};
}

static Color scroll_led(float _time, Color color, float period_1, float period_2) {
  if(_time < 0.175f) return color;
  if(_time < (0.175f + period_1)) return {};
  return color;
}

ActiveLED::ActiveLED()
: color(), active(false), _time(0) {
  }

void ActiveLED::update(Color _color, LedEffect _effect, float _period_1, float _period_2, bool _loop) {
  tcolor = _color;
  period_1 = _period_1;
  period_2 = _period_2;
  loop = _loop;
  effect = _effect;
  period = _period_1 + _period_2;
  if(effect == LedEffect::off) color = {};
  if(effect == LedEffect::on) color = tcolor;
  if(effect == LedEffect::scrolling) period = 0.175 + _period_1 + 7.5 * period_2;
  active = (effect != LedEffect::off && effect != LedEffect::on);
  if(active) _time = 0;
}

void ActiveLED::do_step(float time_step) {
  if(!active) return;
  _time = _time + time_step;
  if(!loop && _time > period) {
    active = false;
    // TODO(jerome): Check which color at the end of an effect
    return;
  }
  _time = fmod(_time, period);
  if(effect == LedEffect::flash) {
    color = flash_led(_time, tcolor, period_1, period_2);
  }
  else if (effect == LedEffect::breath) {
    color = breath_led(_time, tcolor, period_1, period_2);
  }
  else if (effect == LedEffect::scrolling) {
    color = scroll_led(_time, tcolor, period_1, period_2);
  }
}

static WheelSpeeds wheel_speeds_from_twist(Twist2D &twist, float l=0.2, float radius=0.05) {
  WheelSpeeds value;
  value.front_left = (twist.x - twist.y - l * twist.theta) / radius;
  value.front_right = (twist.x + twist.y + l * twist.theta) / radius;
  value.rear_left = (twist.x + twist.y - l * twist.theta) / radius;
  value.rear_right = (twist.x - twist.y + l * twist.theta) / radius;
  return value;
}

static Twist2D twist_from_wheel_speeds(WheelSpeeds &speeds, float l=0.2, float radius=0.05) {
  Twist2D value;
  value.x = 0.25 * (speeds.front_left + speeds.front_right + speeds.rear_left + speeds.rear_right) * radius;
  value.y = 0.25 * (-speeds.front_left + speeds.front_right + speeds.rear_left - speeds.rear_right) * radius;
  value.theta = 0.25 * (-speeds.front_left + speeds.front_right - speeds.rear_left + speeds.rear_right) * radius / l;
  return value;
}

const float L = 0.12f;
// The coordinate system of the arm is located at the middle top of the arm [trapeziodal] platfom
// The end effector is longitudinally at start of edge of the gripper, at an altitude
// about 1 cm below the arm end effector
static ServoValues<float> ARM_ZERO {1.5508f, 2.6578f};

// Defined to that (0.071, 0.057) is the point with maximal flection (like in the real RM)
static Vector3 ARM_OFFSET {-0.0007f, 0.0f, 0.0043f};

// static Vector3 ARM_OFFSET {0.0032f, 0.0f, 0.0179f};

static Vector3 forward_arm_kinematics(ServoValues<float> & _angles) {
  ServoValues<float> angles = ARM_ZERO - _angles;
  Vector3 p = {cos(angles.right) - cos(angles.left), 0, sin(angles.right) - sin(angles.left)};
  return p * L + ARM_OFFSET;
}

static Matrix2 Jacobian(ServoValues<float> angles) {
  return {L * sin(ARM_ZERO.right - angles.right),
          -L * sin(ARM_ZERO.left - angles.left),
          -L * cos(ARM_ZERO.right - angles.right),
          L * cos(ARM_ZERO.left - angles.left)};
}

static ServoValues<float> inverse_arm_kinematics(Vector3 & position, Vector3 & arm_position, ServoValues<float> arm_angles, float k = 0.2) {
  Vector3 diff = position - arm_position;
  Vector2 delta = {diff.x, diff.z};
  Matrix2 m = Jacobian(arm_angles);
  // spdlog::info("Jacobian {}", m);
  m = m.inverse();
  // spdlog::info("Inverse {}", m);
  Vector2 delta_angle = m * delta;
  // spdlog::info("inverse_arm_kinematics {} -> {}", delta, delta_angle);
  ServoValues<float> delta_servo_angles {k * delta_angle.x, k * delta_angle.y};
  ServoValues<float> angles = arm_angles + delta_servo_angles;
  // auto v = forward_arm_kinematics(angles);
  // spdlog::info("forward_arm_kinematics {} ~-> {}", angles, v);
  // if((v - position).norm() > 0.001){
  //   return inverse_arm_kinematics(position, v, angles);
  // }
  // spdlog::info("Converged", angles, v);
  return angles;
}


Robot::Robot()
  : imu(), target_wheel_speed(), target_servo_angles(), target_gripper_state(Robot::GripperStatus::pause),
  mode(Mode::FREE), axis_x(0.1),  axis_y(0.1), wheel_radius(0.05), sdk_enabled(false),
  odometry(), body_twist(), desired_target_wheel_speed(), wheel_speeds(), wheel_angles(),
  leds(), led_colors(), time_(0.0f),
  desired_gripper_state(target_gripper_state),
  commands(nullptr), discovery(nullptr), video_streamer(nullptr), streaming(false),
  servo_angles(), desired_servo_angles()   {
  }

void Robot::do_step(float time_step) {
  time_ += time_step;
  last_time_step = time_step;

  wheel_speeds = read_wheel_speeds();
  wheel_angles = read_wheel_angles();
  update_odometry(time_step);
  imu = read_imu();
  update_attitude(time_step);

  servo_angles = read_servo_angles();
  servo_speeds = read_servo_speeds();
  update_arm_position(time_step);

  gripper_state = read_gripper_state();

  // spdlog::debug("state {}", odometry);


  // Update Actions

  if(move_action) {
    if(move_action->state == Action::State::started) {
        move_action->goal_odom = get_pose() * move_action->goal;
        move_action->state = Action::State::running;
        spdlog::info("Start Move Action to {} [odom]", move_action->goal_odom);
    }
    if(move_action->state == Action::State::running) {
      Pose2D goal = move_action->goal_odom.relative_to(get_pose());
      // Real robomaster is not normalizing!
      // goal.theta = normalize(goal.theta);
      spdlog::info("Update Move Action to {} [frame] from {} [odom]", goal, get_pose());
      const float tau = 0.5f;
      float remaining_duration;
      Twist2D twist;
      if (goal.norm() < 0.01) {
        twist = {0, 0, 0};
        move_action->state = Action::State::succeed;
        remaining_duration = 0;
        spdlog::info("Move Action done", goal);
      }
      else {
        float f = std::min(1.0f, move_action->linear_speed / (goal.distance() / tau));
        twist = {
          f * goal.x / tau,
          f * goal.y / tau,
          std::clamp(goal.theta / tau, -move_action->angular_speed, move_action->angular_speed)
        };
        remaining_duration = time_to_goal(goal, move_action->linear_speed, move_action->angular_speed);
        spdlog::info("Move Action continue [{:.2f} s]", remaining_duration);
      }
      move_action->current = goal;
      move_action->remaining_duration = remaining_duration;
      set_target_velocity(twist);
    }
    auto data = move_action->do_step(time_step);
    if(data.size()) {
      spdlog::debug("Push action {} bytes: {:n}", data.size(), spdlog::to_hex(data));
      commands->send(data);
      if (move_action->state == Action::State::failed || move_action->state == Action::State::succeed) {
        move_action = NULL;
      }
    }
  }

  if(move_arm_action) {
    bool first = false;
    if(move_arm_action->state == Action::State::started) {
      if(move_arm_action->absolute) {

      }
      else {
        move_arm_action->goal_position = get_arm_position() + move_arm_action->goal_position;
      }
      move_arm_action->state = Action::State::running;
      spdlog::info("[Move Arm Action] set goal to {}", move_arm_action->goal_position);
      first = true;
    }
    if(move_arm_action->state == Action::State::running) {
      set_target_arm_position(move_arm_action->goal_position);
      ServoValues<float> angles = get_servo_angles();
      ServoValues<float> target_angles = desired_servo_angles;
      move_arm_action->remaining_duration = (
        abs(normalize(target_angles.right - angles.right)) +
        abs(normalize(target_angles.left - angles.left)));
        if(first) move_arm_action->predicted_duration = move_arm_action->remaining_duration;
        if(move_arm_action->remaining_duration < 0.005) {
          move_arm_action->state = Action::State::succeed;
          move_arm_action->remaining_duration = 0;
          spdlog::info("[Move Arm Action] done");
        }
        else {
          spdlog::info("[Move Arm Action] will continue for {:.2f} rad", move_arm_action->remaining_duration);
        }
    }
    // if(move_arm_action->state == Action::State::running) {
      move_arm_action->current_position = get_arm_position();
      auto distance = (move_arm_action->current_position - move_arm_action->goal_position).norm();
      spdlog::info("[Move Arm Action] current arm position {} is distant {} from goal", move_arm_action->current_position, distance);
    //   if(distance < 0.001) {
    //     move_arm_action->state = Action::State::succeed;
    //     move_arm_action->remaining_duration = 0;
    //     spdlog::info("[Move Arm Action] done");
    //   }
      // else {
      //   spdlog::info("[Move Arm Action] will continue for {:.2f} rad", move_arm_action->remaining_duration);
      // }
    auto data = move_arm_action->do_step(time_step);
    if(data.size()) {
      spdlog::debug("Push action {} bytes: {:n}", data.size(), spdlog::to_hex(data));
      commands->send(data);
      if (move_arm_action->state == Action::State::failed || move_arm_action->state == Action::State::succeed) {
        move_arm_action = NULL;
      }
    }
  }

  if(play_sound_action) {
    play_sound_action->remaining_duration -= time_step;
    play_sound_action->state = Action::State::running;
    if(play_sound_action->remaining_duration < 0) {
      if(play_sound_action->play_times == 0) {
        play_sound_action->state = Action::State::succeed;
      }
      else {
        spdlog::info("[Robot] Play sound {}", play_sound_action->play_times);
        play_sound_action->remaining_duration = play_sound_action->predicted_duration;
        play_sound_action->play_times--;
      }
    }
    auto data = play_sound_action->do_step(time_step);
    if(data.size()) {
      spdlog::debug("Push action {} bytes: {:n}", data.size(), spdlog::to_hex(data));
      commands->send(data);
      if (play_sound_action->state == Action::State::failed || play_sound_action->state == Action::State::succeed) {
        play_sound_action = NULL;
      }
    }
  }

  // for (auto const& [key, action] : actions) {
  //   commands->send(action->encode());
  //   if (action->state == Action::State::failed || action->state == Action::State::succeed) {
  //     actions.erase(action->id);
  //   }
  // }

  // LED
  leds.do_step(time_step);
  LEDColors desired_led_colors = leds.desired_colors();
  if (desired_led_colors != led_colors) {
    spdlog::debug("led_colors -> desired_led_colors = {}", desired_led_colors);
    led_colors = desired_led_colors;
    update_led_colors(led_colors);
  }
  // Motors
  if (desired_target_wheel_speed != target_wheel_speed) {
    spdlog::debug("target_wheel_speed -> desired_target_wheel_speed = {}", desired_target_wheel_speed);
    target_wheel_speed = desired_target_wheel_speed;
    update_target_wheel_speeds(target_wheel_speed);
  }

  // Arm
  if (desired_servo_angles != target_servo_angles) {
    // spdlog::debug("target_servo_angles -> desired_servo_angles = {}", desired_servo_angles);
    target_servo_angles = desired_servo_angles;
    update_target_servo_angles(target_servo_angles);
  }

  if (target_gripper_state != desired_gripper_state || target_gripper_power != desired_gripper_power) {
    target_gripper_state = desired_gripper_state;
    target_gripper_power = desired_gripper_power;
    update_target_gripper(target_gripper_state, target_gripper_power);
  }


  // arm
  // Gripper
  //

  // Update Publishers
  if(commands)
    commands->do_step(time_step);


  // Stream the camera image
  if(streaming) {
    // TODO(jerome): I miss something important with allocation/shifting around of buffers.
    // If I do not copy here -> segfault ... why?
    spdlog::debug("[Robot] capture new camera frame");
    auto image = read_camera_image();
    // std::cout << int(image[0]) << " " << int(image[1]) << " " << int(image[2]) << " (" << image.size() <<")\n";
    // auto image = std::vector<unsigned char>(640 * 360 * 3, 0);
    // auto image = _image;
    // static unsigned seq = 0;
    // seq++;
    // std::string name = "/Users/Jerome/Dev/My/robomaster_simulation/build/images/image" + std::to_string(seq) + ".dat";
    // std::ofstream fout(name.c_str(), std::ios::out | std::ios::binary);
    // fout.write((char *)image.data(), image.size());
    // fout.close();

    if(!image.empty()) {
      video_streamer->send(image.data());
    }
  }

  if(discovery) {
    discovery->do_step(time_step);
  }
}

void Robot::update_odometry(float time_step){
  body_twist = twist_from_wheel_speeds(wheel_speeds);
  odometry.twist = body_twist.rotate_around_z(odometry.pose.theta);
  odometry.pose = odometry.pose + odometry.twist * time_step;
  // spdlog::info("update_odometry {}, {}, {}", body_twist, odometry.twist, odometry.pose);
}

void Robot::update_attitude(float time_step){
  // TODO(jerome): compute attitude from the other sensing values
  imu.attitude.yaw += time_step * imu.angular_velocity.z;

  // use attitude as a source for odometry angular data:
  odometry.twist.theta = imu.angular_velocity.z;
  odometry.pose.theta = imu.attitude.yaw;
  // TODO(jerome): body twist too
  // spdlog::info("update_attitude {}, {}", imu.attitude.yaw,  odometry.pose);
}

void Robot::update_arm_position(float time_step){
  arm_position = forward_arm_kinematics(servo_angles);
}

void Robot::set_target_wheel_speeds(WheelSpeeds &speeds) {
  desired_target_wheel_speed = speeds;
}

void Robot::set_led_effect(Color color, LedMask mask, ActiveLED::LedEffect effect, float period_on, float period_off, bool loop)
{
  if(mask & ARMOR_BOTTOM_BACK) leds.rear.update(color, effect, period_on, period_off, loop);
  if(mask & ARMOR_BOTTOM_FRONT) leds.front.update(color, effect, period_on, period_off, loop);
  if(mask & ARMOR_BOTTOM_LEFT) leds.right.update(color, effect, period_on, period_off, loop);
  if(mask & ARMOR_BOTTOM_RIGHT) leds.left.update(color, effect, period_on, period_off, loop);
}

void Robot::set_mode(Mode _mode){
  mode = _mode;
}

Robot::Mode Robot::get_mode(){
  return mode;
}

void Robot::set_target_velocity(Twist2D &twist)
{
  // TODO(jerome): Could also postpose this at do_step time.
  // should do it anyway for the position control
  WheelSpeeds speeds = wheel_speeds_from_twist(twist, axis_x + axis_y, wheel_radius);
  set_target_wheel_speeds(speeds);
}

void Robot::set_target_servo_angles(ServoValues<float> & values)
{
  // spdlog::info("set_target_servo_angles {}", values);
  // Limits:
  // -0.2740 < right < 1.3840
  // -0.7993 < left (< 1.7313)
  // -0.3473 < right - left < 1.2147
  values.right = std::clamp(values.right, -0.2740f, 1.3840f);
  values.left = std::clamp(values.left, -0.7993f, 1.7313f);
  // distance from the (right - left) upper line
  float dist = (values.right - values.left - 1.2147) / sqrt(2);
  if (dist > 0) {
    // compute nearest point on the line by moving down along (1, -1) by distance
    values.right -= dist;
    values.left += dist;
  }
  // distance from the (right - left) lower line
  dist = (values.right - values.left + 0.3473) / sqrt(2);
  if (dist < 0) {
    // compute nearest point on the line by moving down along (1, -1) by distance
    values.right -= dist;
    values.left += dist;
  }
  // spdlog::info("-> {}", values);
  desired_servo_angles = values;
}

void Robot::set_target_arm_position(Vector3 & position) {
  // no planning, just a dummy (direct) controller based on inverse kinematic
  ServoValues<float> angles = inverse_arm_kinematics(position, arm_position, servo_angles);
  set_target_servo_angles(angles);
}


void Robot::set_enable_sdk(bool value){
  if(sdk_enabled == value) return;
  sdk_enabled = value;
  if(value)
    spdlog::info("Enabled SDK");
  else
    spdlog::info("Disabled SDK");
}

Twist2D Robot::get_twist(Frame frame) {
  if(frame == Frame::odom)
    return odometry.twist;
  else
    return body_twist;
}

Pose2D Robot::get_pose() {
  return odometry.pose;
}

Attitude Robot::get_attitude() {
  return imu.attitude;
}

IMU Robot::get_imu() {
  return imu;
}

Robot::GripperStatus Robot::get_gripper_status() {
  return gripper_state;
}

Vector3 Robot::get_arm_position() {
  return arm_position;
}

void Robot::set_target_gripper(GripperStatus state, float power) {
  desired_gripper_state = state;
  desired_gripper_power = power;
}

bool Robot::submit_action(std::shared_ptr<MoveAction> action) {
  // TODO(jerome): What should we do if an action is already active?
  if(move_action) return false;
  move_action = action;
  move_action->state = Action::State::started;
  return true;
}

bool Robot::submit_action(std::shared_ptr<MoveArmAction> action) {
  // TODO(jerome): What should we do if an action is already active?
  if(move_arm_action) return false;
  move_arm_action = action;
  move_arm_action->state = Action::State::started;
  return true;
}

bool Robot::submit_action(std::shared_ptr<PlaySoundAction> action) {
  // TODO(jerome): What should we do if an action is already active?
  if(play_sound_action) return false;
  play_sound_action = action;
  play_sound_action->state = Action::State::started;
  return true;
}



bool Robot::start_streaming(unsigned width, unsigned height) {
  if(!video_streamer) {
    spdlog::warn("[Robot] has no video streamer to be started");
    return false;
  }
  if(!last_time_step) {
    spdlog::warn("[Robot] time step unknown: cannot set video streamer fps");
    return false;
  }
  camera_width = width;
  camera_height = height;
  if(!set_camera_resolution(width, height)) {
    spdlog::warn("[Robot] Camera resolution {} x {} not supported", width, height);
    return false;
  }
  auto address = commands->sender_endpoint().address();
  spdlog::info("[Robot] Start video streamer to {}", address);
  streaming = true;
  video_streamer->start(address, width, height, 1.0f / last_time_step);
  return true;
}

bool Robot::stop_streaming() {
  if(!video_streamer) {
    spdlog::warn("[Robot] has no video streamer to be stopped");
    return false;
  }
  spdlog::info("[Robot] stop streaming");
  streaming = false;
  video_streamer->stop();
  return true;
}
