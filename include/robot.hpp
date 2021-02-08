#ifndef INCLUDE_ROBOT_HPP_
#define INCLUDE_ROBOT_HPP_

// TODO(jerome): make sure we are not uselessly copying images

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

#include "utils.hpp"

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

inline float time_to_goal(const Pose2D &goal_pose, float linear_speed, float angular_speed) {
  return std::max(abs(normalize(goal_pose.theta)) / angular_speed,
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

  inline bool operator==(const Color &rhs) const { return r == rhs.r && g == rhs.g && b == rhs.b; }

  inline Color operator*(float f) {
    return {std::clamp(r * f, 0.0f, 1.0f), std::clamp(g * f, 0.0f, 1.0f),
            std::clamp(b * f, 0.0f, 1.0f)};
  }

  inline bool operator!=(const Color &rhs) { return !(*this == rhs); }
};

template <typename OStream> OStream &operator<<(OStream &os, const Color &v) {
  os << "Color <" << int(v.r) << ", " << int(v.g) << ", " << int(v.b) << ">";
  return os;
}

class ActiveLED {
 public:
  enum LedEffect { off = 0, on = 1, breath = 2, flash = 3, scrolling = 4 };

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

  template <typename OStream> friend OStream &operator<<(OStream &os, const Attitude &v) {
    os << "Attitude <" << v.yaw << ", " << v.pitch << ", " << v.roll << " >";
    return os;
  }
};

struct IMU {
  Vector3 angular_velocity;
  Vector3 acceleration;

  template <typename OStream> friend OStream &operator<<(OStream &os, const IMU &v) {
    os << "IMU <" << v.angular_velocity << ", " << v.acceleration << " >";
    return os;
  }
};

struct Odometry {
  Pose2D pose;
  Twist2D twist;

  template <typename OStream> friend OStream &operator<<(OStream &os, const Odometry &v) {
    os << "Odom <" << v.pose << ", " << v.twist << " >";
    return os;
  }
};

using WheelSpeeds = WheelValues<float>;
using Image = std::vector<uint8_t>;

struct HitEvent {
  uint8_t type;
  uint8_t index;
};

using hit_event_t = std::vector<HitEvent>;

struct DetectedObjects {
  enum Type : uint8_t { PERSON = 1, GESTURE = 2, LINE = 4, MARKER = 5, ROBOT = 7 };

  struct Person {
    constexpr static Type type{PERSON};
    BoundingBox bounding_box;
    explicit Person(BoundingBox bounding_box)
        : bounding_box(bounding_box) {}
  };

  struct Gesture {
    constexpr static Type type{GESTURE};
    BoundingBox bounding_box;
    uint32_t id;
    Gesture(BoundingBox bounding_box, uint32_t id)
        : bounding_box(bounding_box)
        , id(id) {}
  };

  struct Line {
    constexpr static Type type{LINE};
    float x;
    float y;
    float curvature;
    float angle;
    uint32_t info;
    Line(float x, float y, float curvature, float angle)
        : x(x)
        , y(y)
        , curvature(curvature)
        , angle(angle) {}
  };

  struct Marker {
    constexpr static Type type{MARKER};
    BoundingBox bounding_box;
    uint16_t id;
    uint16_t distance;
    // TODO(I don't know the distance encoding)
    Marker(BoundingBox bounding_box, uint16_t id, float distance)
        : bounding_box(bounding_box)
        , id(id)
        , distance(uint16_t(distance * 1000)) {}
  };

  struct Robot {
    constexpr static Type type{ROBOT};
    BoundingBox bounding_box;
    explicit Robot(BoundingBox bounding_box)
        : bounding_box(bounding_box) {}
  };

  std::vector<Person> people;
  std::vector<Gesture> gestures;
  std::vector<Line> lines;
  std::vector<Marker> markers;
  std::vector<Robot> robots;

  template <typename T> const std::vector<T> &get() const;

  template <> const std::vector<Person> &get() const { return people; }

  template <> const std::vector<Gesture> &get() const { return gestures; }

  template <> const std::vector<Line> &get() const { return lines; }

  template <> const std::vector<Marker> &get() const { return markers; }

  template <> const std::vector<Robot> &get() const { return robots; }
};

struct ToFReading {
  float distance;
  bool active;
};

struct Servo {
  enum Mode { SPEED = 1, ANGLE = 0 };

  constexpr static float MAX_SPEED = angular_speed_from_rpm(49);

  inline float distance() { return abs(normalize(angle - desired_angle)); }

  float angle;
  float target_angle;
  float desired_angle;
  float speed;
  float target_speed;
  float desired_speed;
  Mode mode;
  Mode desired_mode;
  bool enabled;
};

using Servos = ServoValues<Servo>;

ServoValues<float> limit_servo_angles(const ServoValues<float> &target_values);

class Robot {
  friend struct MoveAction;
  friend struct MoveArmAction;
  friend struct PlaySoundAction;
  friend struct MoveServoAction;

 public:
  struct Camera {
    int width;
    int height;
    float fps;
    bool streaming;
    Image image;
    Camera()
        : streaming(false) {}
  };

  struct Vision {
    uint8_t enabled;
    enum Color { RED = 1, GREEN = 2, BLUE = 3 };
    std::map<DetectedObjects::Type, Color> color;
    template <typename T> bool is_enabled() { return (1 << T::type) & enabled; }
    DetectedObjects detected_objects;
  };

  enum Led {
    ARMOR_BOTTOM_BACK = 0x1,
    ARMOR_BOTTOM_FRONT = 0x2,
    ARMOR_BOTTOM_LEFT = 0x4,
    ARMOR_BOTTOM_RIGHT = 0x8,
    ARMOR_TOP_LEFT = 0x10,
    ARMOR_TOP_RIGHT = 0x20,
  };

  using LedMask = uint8_t;
  using LEDColors = LEDValues<Color>;

  struct LED : LEDValues<ActiveLED> {
    inline LEDColors desired_colors() { return {front.color, left.color, rear.color, right.color}; }

    void do_step(float time_step) {
      rear.do_step(time_step);
      front.do_step(time_step);
      left.do_step(time_step);
      right.do_step(time_step);
    }
  };

  enum Mode { FREE = 0, GIMBAL_LEAD = 1, CHASSIS_LEAD = 2 };

  enum GripperStatus { pause = 0, open = 1, close = 2 };

  enum Frame { odom = 0, body = 1 };

  Robot();

  virtual void update_led_colors(LEDColors &) = 0;
  virtual void update_target_wheel_speeds(WheelSpeeds &) = 0;
  virtual WheelSpeeds read_wheel_speeds() = 0;
  virtual WheelValues<float> read_wheel_angles() = 0;
  virtual IMU read_imu() = 0;
  // virtual void control_gripper(GripperStatus state, float power) = 0;

  virtual void do_step(float time_step);

  void update_odometry(float time_step);
  void update_attitude(float time_step);
  /**
   * Set the angular speed of the wheels in [rad/s] with respect to robot y-axis
   */
  void set_target_wheel_speeds(const WheelSpeeds &speeds);

  /**
   * Set the current LED effect
   * @param color      Color
   * @param mask       LED to control
   * @param effect     LED gesture
   * @param period_on  Gesture on-period [DONE(jerome): check]
   * @param period_off Gesture off-period [DONE(jerome): check]
   * @param loop       Repeat the gesture
   */
  void set_led_effect(Color color, LedMask mask, ActiveLED::LedEffect effect, float period_on,
                      float period_off, bool loop);

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
  void set_target_velocity(const Twist2D &twist);

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
  float get_time() { return time_; }

  WheelSpeeds get_wheel_speeds() { return wheel_speeds; }

  WheelValues<float> get_wheel_angles() { return wheel_angles; }

  Action::State submit_action(std::unique_ptr<MoveAction> action);
  Action::State submit_action(std::unique_ptr<MoveArmAction> action);
  Action::State submit_action(std::unique_ptr<PlaySoundAction> action);
  Action::State submit_action(std::unique_ptr<MoveServoAction> action);

  bool start_streaming(unsigned width, unsigned height);
  bool stop_streaming();

  virtual bool set_camera_resolution(unsigned width, unsigned height) = 0;

  virtual Image read_camera_image() = 0;

  void set_target_servo_angles(const ServoValues<float> &angles);
  void set_target_servo_angle(size_t index, float angle);
  void set_target_servo_speed(size_t index, float speed);
  void set_servo_mode(size_t index, Servo::Mode mode);
  void enable_servo(size_t index, bool value);
  float get_servo_angle(size_t index) { return servos[index].angle; }
  float get_servo_speed(size_t index) { return servos[index].speed; }
  ServoValues<float> get_servo_angles() {
    return {.left = servos.left.angle, .right = servos.right.angle};
  }
  virtual void update_servo_mode(size_t index, Servo::Mode mode) = 0;

  void control_arm_position(const Vector3 &position);

  virtual void update_target_servo_angle(size_t index, float angle) = 0;
  virtual void update_target_servo_speed(size_t index, float speed) = 0;
  virtual float read_servo_angle(size_t index) = 0;
  virtual float read_servo_speed(size_t index) = 0;

  void update_arm_position(float time_step);

  virtual DetectedObjects read_detected_objects() = 0;

  void set_enable_vision(uint8_t value) { vision.enabled = value; }

  uint8_t get_enable_vision() { return vision.enabled; }

  const DetectedObjects &get_detected_objects() { return vision.detected_objects; }

  Camera *get_camera() { return &camera; }

  Action::State move_base(const Pose2D &pose, float linear_speed, float angular_speed);
  Action::State move_arm(float x, float z, bool absolute);
  Action::State play_sound(uint32_t sound_id, uint8_t times);
  Action::State move_servo(uint8_t id, float target_angle);

  void add_callback(Callback callback) { callbacks.push_back(callback); }

  hit_event_t get_hit_events() { return hit_events; }

  virtual hit_event_t read_hit_events() = 0;

  // ignored for now:
  // color {1: red, 2: green, 3: blue}
  // type {1: line, 2: marker}
  // TODO(jerome): attention that this enum is different that the one used in vision type
  void set_vision_color(uint8_t type, uint8_t _color) {
    Vision::Color color = static_cast<Vision::Color>(_color);
    if (type == 1)
      vision.color[DetectedObjects::Type::LINE] = color;
    else if (type == 2)
      vision.color[DetectedObjects::Type::MARKER] = color;
  }

  const std::vector<ToFReading> &get_tof_readings() { return tof_readings; }
  virtual std::vector<ToFReading> read_tof() = 0;

 protected:
  IMU imu;
  Attitude attitude;
  Camera camera;
  Vision vision;
  WheelSpeeds target_wheel_speed;
  ServoValues<Servo> servos;
  GripperStatus target_gripper_state;
  float target_gripper_power;
  float last_time_step;
  WheelValues<float> wheel_angles;

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
  LED leds;
  LEDColors led_colors;

  float time_;

  GripperStatus gripper_state;
  GripperStatus desired_gripper_state;
  // float gripper_power;
  float desired_gripper_power;
  Vector3 arm_position;

  std::vector<Callback> callbacks;

  std::map<std::string, std::unique_ptr<Action>> actions;

  hit_event_t hit_events;

  std::vector<ToFReading> tof_readings;
};

#endif  // INCLUDE_ROBOT_HPP_
