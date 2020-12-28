#ifndef ROBOT_HPP
#define ROBOT_HPP

struct Color {
  unsigned char r;
  unsigned char g;
  unsigned char b;
};

struct Pose2D {
  float x, y, theta;
};

struct Twist2D {
  float x, y, theta;
};

struct Attitude {
  float yaw, pitch, roll;
};

struct Vector3 {
  float x, y, z;
};

struct IMU {
  Attitude attitude;
  Vector3 angular_velocity;
  Vector3 acceleration;
};


class Robot
{
public:
  enum LedEffect {
    off = 0,
    on = 1,
    breath = 2,
    flash = 3,
    scrolling = 4
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


  Robot()
  : mode(Mode::FREE){

  }

  /**
  * Set the angular speed of the wheels in [rad/s] with respect to robot y-axis
  * @param front_right front right angular wheel speed
  * @param front_left  front left angular wheel speed
  * @param rear_left   rear left angular wheel speed
  * @param rear_right  rear right angular wheel speed
  */
  virtual void set_wheel_speeds(float front_right, float front_left, float rear_left, float rear_right) = 0;

  /**
   * [set_leds description]
   * @param color      Color
   * @param mask       LEDs to control
   * @param effect     LEDs gesture
   * @param period_on  Gesture on-period [TODO(jerome): check]
   * @param period_off Gesture off-period [TODO(jerome): check]
   * @param loop       Repeat the gesture
   */
  virtual void set_leds(Color color, LedMask mask, LedEffect effect, float period_on, float period_off, bool loop) = 0;

  /**
   * Set the type of coordination between gimbal and chassis
   * @param mode The desired robot mode
   */
  void set_mode(Mode _mode){
    mode = _mode;
  }

  /**
   * Get the type of coordination between gimbal and chassis
   * @return The current robot mode
   */
  Mode get_mode(){
    return mode;
  }

  /**
   * Set the robot velocity in its frame.
   * @param x     Linear velocity (x, [m/s])
   * @param y     Linear velocity (y, [m/s])
   * @param theta Angualar velocity (theta, [rad/s])
   */
  virtual void set_velocity(float x, float y, float theta) = 0;

  /**
   * Enable/disable the SDK [client]
   * @param value SDK [client] state
   */
  virtual void set_enable_sdk(bool value) = 0;

  /**
   * Control the gripper
   * @param state the desired state: open, close or pause the gripper
   * @param power The fraction of power to apply
   */
  virtual void control_gripper(GripperStatus state, float power) = 0;

  Twist2D get_twist(Frame frame) { return {}; };

  Pose2D get_pose() { return {}; };

  Attitude get_attitude() { return {}; };

  IMU get_imu() { return {}; };

  GripperStatus get_gripper_status() { return GripperStatus::pause; };

  Vector3 get_arm_position() { return {}; };

protected:
  Mode mode;

};

#endif /* end of include guard: ROBOT_HPP */
