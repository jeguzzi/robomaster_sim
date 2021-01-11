#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <math.h>
#include <string.h>
#include <stdexcept>
#include <algorithm>
#include <map>

#include <spdlog/spdlog.h>
#include "spdlog/fmt/ostr.h"

#include "utils.hpp"

struct Action;
struct MoveAction;
struct MoveArmAction;


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

class Commands;
class VideoStreamer;
class Discovery;

class Robot
{
public:

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

  void set_commands(Commands * _commands) {
    commands = _commands;
  }

  void set_discovery(Discovery * _discovery) {
    discovery = _discovery;
  }

  void set_video_streamer(VideoStreamer * _video) {
    video_streamer = _video;
  }

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

  //TODO(jerome): use enum for resolution, pass address and protocol
  bool start_streaming(unsigned width, unsigned height);
  bool stop_streaming();

  virtual bool set_camera_resolution(unsigned width, unsigned height) = 0;

  virtual std::vector<unsigned char> read_camera_image() = 0;

  void set_target_servo_angles(ServoValues<float> &angles);

  ServoValues<float> get_servo_angles() {
    return servo_angles;
  }

  void set_target_arm_position(Vector3 &position);

  virtual void update_target_servo_angles(ServoValues<float> &angles) = 0;
  virtual ServoValues<float> read_servo_angles() = 0;

  void update_arm_position(float time_step);

protected:
  IMU imu;
  WheelSpeeds target_wheel_speed;
  ServoValues<float> target_servo_angles;

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
  Commands * commands;
  Discovery * discovery;
  VideoStreamer * video_streamer;
  bool streaming;
  float last_time_step;
  ServoValues<float> servo_angles;
  ServoValues<float> desired_servo_angles;
  // std::map<int, std::shared_ptr<Action>> actions;
  std::shared_ptr<MoveAction> move_action;
  std::shared_ptr<MoveArmAction> move_arm_action;
  // std::shared_ptr<VideoStreamer> video_streamer;
};

#endif /* end of include guard: ROBOT_HPP */
