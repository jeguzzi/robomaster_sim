#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <math.h>
#include <memory>
#include <string.h>
#include <stdexcept>
#include <algorithm>
#include <map>

#include <spdlog/spdlog.h>
#include "spdlog/fmt/ostr.h"

#include "utils.hpp"

class Robot;

typedef std::function<void(float)> Callback;

struct Action
{
  enum State : uint8_t {
    running = 0,
    succeed = 1,
    failed = 2,
    started = 3,
    undefined = 4,
  };
  Action(Robot *robot) : robot(robot), state(State::undefined), callback([](float){}) {};
  virtual void do_step(float time_step) = 0;
  void do_step_cb(float time_step) {
    do_step(time_step);
    callback(time_step);
  }
  virtual ~Action() {};
  bool done() {
    return state == Action::State::failed || state == Action::State::succeed;
  }
  void set_callback(Callback value) {
    callback = value;
  }

  Robot * robot;
  State state;
  float predicted_duration;
  float remaining_duration;
  Callback callback;
};


inline float time_to_goal(Pose2D & goal_pose, float linear_speed, float angular_speed) {
  return std::max(abs(normalize(goal_pose.theta)) / angular_speed, goal_pose.distance() / linear_speed);
}


struct MoveAction : Action
{
  MoveAction(Robot * robot, Pose2D goal_pose, float _linear_speed, float _angular_speed)
  : Action(robot), goal(goal_pose), linear_speed(_linear_speed), angular_speed(_angular_speed) {
    predicted_duration = time_to_goal(goal_pose, linear_speed, angular_speed);
  }
  virtual void do_step(float time_step);
  Pose2D goal;
  Pose2D goal_odom;
  Pose2D current;
  float linear_speed;
  float angular_speed;
};


struct MoveArmAction : Action
{
  MoveArmAction(Robot * robot, float x, float z, bool _absolute)
  : Action(robot), goal_position({x, 0, z}), absolute(_absolute) {
  }

  virtual void do_step(float time_step);
  Vector3 goal_position;
  bool absolute;
  // Vector3 current_position;
};


struct PlaySoundAction : Action
{

  static constexpr float duration = 3.0;

  PlaySoundAction(Robot * robot, uint32_t _sound_id, uint8_t _play_times)
  : Action(robot), sound_id(_sound_id), play_times(_play_times) {
    predicted_duration = duration;
    remaining_duration = 0.0;
  };
  virtual void do_step(float time_step);
  uint32_t sound_id;
  uint8_t play_times;
};



// static unsigned char multiply(unsigned char value, float factor)
// {
//   float v = (float) value * factor;
//   if(v >= 255) return 255;
//   if(v<=0) return 0;
//   return (unsigned char) round(v * 255);
// }

// static float clamp(float value, float factor)
// {
//   float v = (float) value * factor;
//   if(v >= 255) return 255;
//   if(v<=0) return 0;
//   return (unsigned char) round(v * 255);
// }

struct Color {
  // [0, 1]
  float r;
  float g;
  float b;

  inline bool operator==(const Color& rhs) const {
    return r == rhs.r && g == rhs.g && b == rhs.b;
  };

  inline Color operator*(float f) {
    return {std::clamp(r * f, 0.0f, 1.0f), std::clamp(g * f, 0.0f, 1.0f), std::clamp(b * f, 0.0f, 1.0f)};
  }

  inline bool operator!=(const Color& rhs) {
    return !(*this == rhs);
  }
};

template<typename OStream>
OStream& operator<<(OStream& os, const Color& v)
{
  os << "Color <"
     << int(v.r) << ", "
     << int(v.g) << ", "
     << int(v.b)
     << ">";
  return os;
}


class ActiveLED {

public:

  enum LedEffect {
    off = 0,
    on = 1,
    breath = 2,
    flash = 3,
    scrolling = 4
  };

  ActiveLED();

  void update(Color _color, LedEffect _effect, float _period_1, float _period_2, bool _loop);

  void do_step(float time_step);
  Color color;
private:
  bool active;
  Color tcolor;
  LedEffect effect;
  float period_1;
  float period_2;
  float period;
  bool loop;
  float _time;
};


struct Attitude {
  float yaw, pitch, roll;
};

struct IMU {
  Attitude attitude;
  Vector3 angular_velocity;
  Vector3 acceleration;
};

struct Odometry {
  Pose2D pose;
  Twist2D twist;

  template<typename OStream>
  friend OStream& operator<<(OStream& os, const Odometry& v)
  {
    os << "Odom <" << v.pose << ", " << v.twist << " >";
    return os;
  }

};

typedef WheelValues<float> WheelSpeeds;
typedef std::vector<unsigned char> Image;
typedef std::vector<std::shared_ptr<Detection::Object>> detected_items_t;
typedef std::map<uint16_t, detected_items_t> detection_t;

struct HitEvent {
  uint8_t type;
  uint8_t index;
};

typedef std::vector<HitEvent> hit_event_t;


class Robot
{

friend struct MoveAction;
friend struct MoveArmAction;
friend struct PlaySoundAction;

public:

  struct Camera {
    int width;
    int height;
    float fps;
    bool streaming;
    Image image;
    Camera() : streaming(false) {};
  };

  enum Led {
    ARMOR_BOTTOM_BACK = 0x1,
    ARMOR_BOTTOM_FRONT = 0x2,
    ARMOR_BOTTOM_LEFT = 0x4,
    ARMOR_BOTTOM_RIGHT = 0x8,
    ARMOR_TOP_LEFT = 0x10,
    ARMOR_TOP_RIGHT = 0x20,
  };

  typedef unsigned char LedMask;
  typedef LEDValues<Color> LEDColors;

  struct LED : LEDValues<ActiveLED> {

    inline LEDColors desired_colors()
    {
      return {front.color, left.color, rear.color, right.color};
    }

    void do_step(float time_step) {
      rear.do_step(time_step);
      front.do_step(time_step);
      left.do_step(time_step);
      right.do_step(time_step);
    }

  };


  enum Mode {
    FREE = 0,
    GIMBAL_LEAD = 1,
    CHASSIS_LEAD = 2
  };

  enum GripperStatus {
    pause = 0,
    open = 1,
    close = 2
  };

  enum Frame {
    odom = 0,
    body = 1
  };

  Robot();

  virtual void update_led_colors(LEDColors &) = 0;
  virtual void update_target_wheel_speeds(WheelSpeeds &) = 0;
  virtual WheelSpeeds read_wheel_speeds() = 0;
  virtual WheelValues<float> read_wheel_angles() = 0;
  virtual IMU read_imu() = 0;
  // virtual void control_gripper(GripperStatus state, float power) = 0;

  void do_step(float time_step);

  void update_odometry(float time_step);
  void update_attitude(float time_step);
  /**
  * Set the angular speed of the wheels in [rad/s] with respect to robot y-axis
  */
  void set_target_wheel_speeds(WheelSpeeds &speeds);

  /**
   * Set the current LED effect
   * @param color      Color
   * @param mask       LED to control
   * @param effect     LED gesture
   * @param period_on  Gesture on-period [TODO(jerome): check]
   * @param period_off Gesture off-period [TODO(jerome): check]
   * @param loop       Repeat the gesture
   */
  void set_led_effect(Color color, LedMask mask, ActiveLED::LedEffect effect, float period_on, float period_off, bool loop);

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
   * Set the robot velocity in its frame.
   */
  void set_target_velocity(Twist2D &twist);

  /**
   * Enable/disable the SDK [client]
   * @param value SDK [client] state
   */
  void set_enable_sdk(bool value);



  Twist2D get_twist(Frame frame);

  Pose2D get_pose();

  Attitude get_attitude();

  IMU get_imu();

  GripperStatus get_gripper_status();

  Vector3 get_arm_position();

  /**
   * Control the gripper
   * @param state the desired state: open, close or pause the gripper
   * @param power The fraction of power to apply
   */
  void set_target_gripper(GripperStatus state, float power);

  virtual GripperStatus read_gripper_state() = 0;

  virtual void update_target_gripper(GripperStatus state, float power) = 0;


  // in seconds
  float get_time() {
    return time_;
  }


  WheelSpeeds get_wheel_speeds() {
    return wheel_speeds;
  }

  WheelValues<float> get_wheel_angles() {
    return wheel_angles;
  }

  bool submit_action(std::shared_ptr<MoveAction> action);
  bool submit_action(std::shared_ptr<MoveArmAction> action);
  bool submit_action(std::shared_ptr<PlaySoundAction> action);

  //TODO(jerome): use enum for resolution, pass address and protocol
  bool start_streaming(unsigned width, unsigned height);
  bool stop_streaming();

  virtual bool set_camera_resolution(unsigned width, unsigned height) = 0;

  virtual Image read_camera_image() = 0;

  void set_target_servo_angles(ServoValues<float> &angles);

  ServoValues<float> get_servo_angles() {
    return servo_angles;
  }

  ServoValues<float> get_servo_speeds() {
    return servo_speeds;
  }

  void set_target_arm_position(Vector3 &position);

  virtual void update_target_servo_angles(ServoValues<float> &angles) = 0;
  virtual ServoValues<float> read_servo_angles() = 0;
  virtual ServoValues<float> read_servo_speeds() = 0;


  void update_arm_position(float time_step);

  virtual detection_t read_detected_objects() = 0;

  void set_enable_vision(uint8_t value) {
    enabled_vision = value;
  }

  uint8_t get_enable_vision() {
    return enabled_vision;
  }

  detection_t * get_detected_objects() {
    return &detected_objects;
  }

  Camera * get_camera() {
    return &camera;
  }

  bool move(Pose2D pose, float linear_speed, float angular_speed);
  bool move_arm(float x, float z, bool absolute);
  bool play_sound(uint32_t sound_id, uint8_t times);

  void add_callback(Callback callback) {
    callbacks.push_back(callback);
  }

  hit_event_t get_hit_events() {
    return hit_events;
  }

  virtual hit_event_t read_hit_events() = 0;

  // ignored for now:
  // color {1: red, 2: green, 3: blue}
  // type {1: line, 2: marker}
  // TODO: attention that this enum is different that the one used in vision type
  void set_vision_color(uint8_t type, uint8_t color) {
    vision_color[type] = color;
  }

protected:
  IMU imu;
  Camera camera;
  WheelSpeeds target_wheel_speed;
  ServoValues<float> target_servo_angles;
  GripperStatus target_gripper_state;
  float target_gripper_power;
  float last_time_step;
  uint8_t enabled_vision;
  std::map<uint8_t, uint8_t> vision_color;

private:
  Mode mode;

  float axis_x;
  float axis_y;
  float wheel_radius;

  bool sdk_enabled;

  Odometry odometry;
  Twist2D body_twist;
  WheelSpeeds desired_target_wheel_speed;
  WheelSpeeds wheel_speeds;
  WheelValues<float> wheel_angles;
  LED leds;
  LEDColors led_colors;

  float time_;

  GripperStatus gripper_state;
  GripperStatus desired_gripper_state;
  //float gripper_power;
  float desired_gripper_power;
  Vector3 arm_position;

  std::vector<Callback> callbacks;

  ServoValues<float> servo_angles;
  ServoValues<float> desired_servo_angles;
  ServoValues<float> servo_speeds;
  std::map<std::string, std::shared_ptr<Action>> actions;

  detection_t detected_objects;
  hit_event_t hit_events;
  // std::shared_ptr<VideoStreamer> video_streamer;
};

#endif /* end of include guard: ROBOT_HPP */
