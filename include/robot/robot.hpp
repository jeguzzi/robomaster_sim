#ifndef INCLUDE_ROBOT_ROBOT_HPP_
#define INCLUDE_ROBOT_ROBOT_HPP_

// TODO(Jerome): make sure we are not uselessly copying images

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "spdlog/spdlog.h"

#include "spdlog/fmt/ostr.h"

#include "../utils.hpp"

#include "action.hpp"
#include "arm.hpp"
#include "armor.hpp"
#include "blaster.hpp"
#include "camera.hpp"
#include "chassis.hpp"
#include "gimbal.hpp"
#include "gripper.hpp"
#include "led.hpp"
#include "servo.hpp"
#include "tof.hpp"
#include "vision.hpp"

// 3 EP servos + 2 S1 gimbal servos
using Servos = std::array<Servo, 3>;

class Robot {
 public:
  enum Mode { FREE = 0, GIMBAL_LEAD = 1, CHASSIS_LEAD = 2 };

  friend struct MoveAction;
  friend struct MoveArmAction;
  friend struct PlaySoundAction;
  friend struct MoveServoAction;
  friend struct MoveGimbalAction;

  explicit Robot(bool has_arm = false, bool has_gripper = false, ServoValues<bool> has_servo = {},
                 bool has_gimbal = false, bool has_camera = false, bool has_tof = false);

  virtual WheelSpeeds read_wheel_speeds() const = 0;
  virtual void forward_target_wheel_speeds(const WheelSpeeds &) = 0;
  virtual WheelValues<float> read_wheel_angles() const = 0;
  virtual void forward_engage_wheel_motors(bool value) {}
  virtual IMU read_imu() const = 0;
  virtual void forward_chassis_led(size_t index, const Color &) = 0;
  virtual void forward_gimbal_led(size_t index, size_t part, const Color &) = 0;
  virtual Gripper::Status read_gripper_state() const = 0;
  virtual void forward_target_gripper(Gripper::Status state, float power) = 0;
  virtual bool forward_camera_resolution(unsigned width, unsigned height) = 0;
  virtual Image read_camera_image() const = 0;
  virtual hit_event_t read_hit_events() const = 0;
  virtual ir_event_t read_ir_events() const = 0;
  virtual DetectedObjects read_detected_objects() const = 0;
  virtual void forward_target_servo_angle(size_t index, float angle) = 0;
  virtual void forward_target_servo_speed(size_t index, float speed) = 0;
  virtual float read_servo_angle(size_t index) const = 0;
  virtual float read_servo_speed(size_t index) const = 0;
  virtual void forward_servo_mode(size_t index, Servo::Mode mode) = 0;
  virtual void forward_servo_enabled(size_t index, bool value) = 0;
  virtual float read_tof(size_t index) const = 0;
  virtual void forward_blaster_led(float value) const = 0;

  void engage_wheel_motors(bool value) {
    if (value != chassis.wheel_motors_engaged) {
      spdlog::info("[Chassis] Set motor engaged to {}", value);
      chassis.wheel_motors_engaged = value;
      forward_engage_wheel_motors(value);
      if (value) {
        chassis.stop();
      }
    }
  }

  bool enable_tof(size_t index) {
    if (tof.set_enable(index, true)) {
      has_tof = true;
      return true;
    }
    return false;
  }

  void read_chassis();
  void control_chassis();
  void control_leds();
  void control_servos();
  void control_gimbal();
  virtual void do_step(float time_step);

  bool start_streaming(unsigned width, unsigned height);
  bool stop_streaming();

  void set_target_servo_angle(size_t index, float angle) {
    servos[index].angle.desired = angle;
    servos[index].mode.desired = Servo::ANGLE;
  }
  void set_target_servo_speed(size_t index, float speed) { servos[index].speed.desired = speed; }
  void set_servo_mode(size_t index, Servo::Mode value) { servos[index].mode.desired = value; }
  void enable_servo(size_t index, bool value) { servos[index].enabled.desired = value; }
  float get_servo_angle(size_t index) { return servos[index].angle.current; }
  float get_servo_speed(size_t index) { return servos[index].speed.current; }

  ServoValues<float> get_servo_angles() {
    return {servos[0].angle.current, servos[1].angle.current, servos[2].angle.current};
  }

  Camera *get_camera() { return &camera; }
  const ToFReadings & get_tof_readings() { return tof.readings; }
  /**
   * Set the current LED effect
   * @param color      Color
   * @param mask       LED to control
   * @param led_mask   the indices of the individual LEDs composing the LED to control
   * @param effect     LED gesture
   * @param period_on  Gesture on-period [DONE(jerome): check]
   * @param period_off Gesture off-period [DONE(jerome): check]
   * @param loop       Repeat the gesture
   */
  void set_led_effect(Color color, LedMask mask, CompositeLedMask led_mask,
                      ActiveLED::LedEffect effect, float period_on, float period_off, bool loop);

  /**
   * Set the type of coordination between gimbal and chassis
   * @param mode The desired robot mode
   */
  void set_mode(Mode _mode);
  /**
   * Get the type of coordination between gimbal and chassis
   * @return The current robot mode
   */
  Mode get_mode();

  /**
   * Enable/disable the SDK [client]
   * @param value SDK [client] state
   */
  void set_enable_sdk(bool value);

  void set_blaster_led(float intensity, bool on) {
    blaster_led.intensity.desired = (on ? intensity : 0.0);
  }

  // in seconds
  float get_time() { return time_; }

  Action::State submit_action(std::unique_ptr<MoveAction> action);
  Action::State submit_action(std::unique_ptr<MoveArmAction> action);
  Action::State submit_action(std::unique_ptr<PlaySoundAction> action);
  Action::State submit_action(std::unique_ptr<MoveServoAction> action);
  Action::State submit_action(std::unique_ptr<MoveGimbalAction> action);

  Action::State move_base(const Pose2D &pose, float linear_speed, float angular_speed);
  Action::State move_arm(float x, float z, bool absolute);
  Action::State play_sound(unsigned sound_id, unsigned times);
  Action::State move_servo(size_t id, float target_angle);
  Action::State move_gimbal(float target_yaw, float target_pitch, float yaw_speed,
                            float pitch_speed, Gimbal::Frame yaw_frame, Gimbal::Frame pitch_frame);

  void add_callback(Callback callback) { callbacks.push_back(callback); }

  Armor armor;
  Servos servos;
  Arm arm;
  BlasterLED blaster_led;
  Camera camera;
  Chassis chassis;
  ChassisLED chassis_leds;
  Gimbal gimbal;
  GimbalLED gimbal_leds;
  Gripper gripper;
  ToF tof;
  Vision vision;

  Action *get_action(std::string name) {
    if (actions.count(name)) {
      return actions[name].get();
    }
    return nullptr;
  }

  Action::State get_action_state(std::string name) {
    if (actions.count(name)) {
      return actions[name]->state;
    }
    if (previous_action_state.count(name)) {
      return previous_action_state[name];
    }
    return Action::State::undefined;
  }

 protected:
  bool has_arm;
  bool has_gripper;
  ServoValues<bool> has_servo;
  bool has_gimbal;
  bool has_camera;
  bool has_tof;
  float last_time_step;
  std::map<unsigned, Servo *> connected_servos;

 private:
  Mode mode;
  bool sdk_enabled;
  float time_;
  std::vector<Callback> callbacks;
  std::map<std::string, std::unique_ptr<Action>> actions;
  std::map<std::string, Action::State> previous_action_state;
};

#if FMT_VERSION >= 90000 
template <> struct fmt::formatter<Robot::Mode>: formatter<std::string_view> {
  auto format(Robot::Mode value, format_context& ctx) const {
  std::string_view name = "";
  switch (value) {
    case Robot::Mode::FREE:   name = "FREE"; break;
    case Robot::Mode::GIMBAL_LEAD: name = "GIMBAL_LEAD"; break;
    case Robot::Mode::CHASSIS_LEAD:  name = "CHASSIS_LEAD"; break;
  }
  return formatter<string_view>::format(name, ctx);    
  }
};
#endif

#endif  // INCLUDE_ROBOT_ROBOT_HPP_
