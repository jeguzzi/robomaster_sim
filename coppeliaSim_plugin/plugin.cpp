#include "plugin.h"

#include <cstdlib>

#include "config.h"
#include "coppeliasim_robot.hpp"
#include "robomaster.hpp"
#include "simPlusPlus/Plugin.h"
#include "spdlog/spdlog.h"
#include "stubs.h"

CS_Vector3 to_cs(const Vector3 &value) {
  return CS_Vector3(value.x, value.y, value.z);
}

CS_Attitude to_cs(const Attitude &value) {
  return CS_Attitude(value.yaw, value.pitch, value.roll);
}

CS_IMU to_cs(const IMU &value) {
  return CS_IMU(to_cs(value.angular_velocity), to_cs(value.acceleration));
}

CS_Pose2D to_cs(const Pose2D &value) {
  return CS_Pose2D(value.x, value.y, value.theta);
}

CS_Twist2D to_cs(const Twist2D &value) {
  return CS_Twist2D(value.x, value.y, value.theta);
}

CS_Odometry to_cs(const Odometry &value) {
  return CS_Odometry(to_cs(value.pose), to_cs(value.twist));
}

static int next_robot_handle = 0;
static std::map<int, std::unique_ptr<CoppeliaSimRobot>> _robots;
static std::map<int, std::unique_ptr<RoboMaster>> _interfaces;

// static boost::asio::io_context io_context;

static const WheelValues<std::string> wheel_names = {
    .front_left = "front_left_wheel_joint",
    .front_right = "front_right_wheel_joint",
    .rear_left = "rear_left_wheel_joint",
    .rear_right = "rear_right_wheel_joint"};

static const ChassisLEDValues<std::string> chassis_led_names = {
    .front = "front_led_link_visual",
    .left = "left_led_link_visual",
    .rear = "rear_led_link_visual",
    .right = "right_led_link_visual"};

static const GimbalLEDValues<std::string> gimbal_led_names = {
    .top_left = "gimbal_left_led_", .top_right = "gimbal_right_led_"};

static const GimbalValues<std::string> gimbal_servo_names = {
    .yaw = "gimbal_yaw_motor", .pitch = "gimbal_pitch_motor"};

static const char servo_name[] = "servo_motor_";
static const char camera_name[] = "Vision_sensor";
static const char vision_name[] = "Machine_vision_sensor";
static const char blaster_light_name[] = "blaster_light";
static const char gripper_name[] = "gripper_link_respondable";
static const char gyro_name[] = "GyroSensor";
static const char accelerometer_name[] = "Accelerometer";

#if SIM_PROGRAM_VERSION_NB < 40300

static int get_index(int handle) {
  const char *name = simGetObjectName(handle);
  return simGetNameSuffix(name);
}

static int get_handle(std::string name, int handle) {
  std::string suffix = "#";
  const int index = get_index(handle);
  if (index >= 0) {
    suffix += std::to_string(index);
  }
  name = name + suffix + "@silentError";
  spdlog::info("Getting handle of {}", name);
  return simGetObjectHandle(name.data());
}

#else

static int get_handle(std::string name, int handle) {
  name = "./" + name;
  spdlog::info("Getting handle of {} in {}", name, handle);
  return simGetObject(name.data(), -1, handle, 1);
}

#endif

static std::string get_ip(std::string network, unsigned *prefix_len) {
  size_t pos = network.find_first_of('/');
  std::string ip = network.substr(0, pos);
  if (pos != std::string::npos) {
    *prefix_len = std::stoul(network.substr(pos + 1));
  } else {
    *prefix_len = 0;
  }
  return ip;
}

static int add_robot(int cs_handle, std::string serial_number,
                     std::string remote_api_network = "",
                     bool enable_camera = true, bool camera_use_udp = false,
                     int camera_bitrate = 1000000, bool enable_arm = true,
                     bool enable_gripper = true, bool enable_gimbal = true,
                     bool enable_vision = true) {
  int handle = next_robot_handle;
  int camera_handle = -1;
  int vision_handle = -1;
  if (enable_camera) {
    camera_handle = get_handle(camera_name, cs_handle);
    if (camera_handle == -1) {
      spdlog::error("Could not locate camera");
      enable_camera = false;
    }
  }
  if (enable_camera && enable_vision) {
    vision_handle = get_handle(vision_name, cs_handle);
  }

  WheelValues<int> wheel_handles;

  for (size_t i = 0; i < wheel_handles.size; i++) {
    int h = get_handle(wheel_names[i], cs_handle);
    if (h == -1) {
      spdlog::error("Could not locate wheel #{}", i);
      return -1;
    }
    wheel_handles[i] = h;
  }

  ChassisLEDValues<int> chassis_led_handles;
  for (size_t i = 0; i < chassis_led_handles.size; i++) {
    int h = get_handle(chassis_led_names[i], cs_handle);
    if (h == -1) {
      spdlog::error("Could not locate led #{}", i);
      return -1;
    }
    chassis_led_handles[i] = h;
  }

  ServoValues<int> servo_motors;
  for (size_t i = 0; i < servo_motors.size(); i++) {
    servo_motors[i] = get_handle(servo_name + std::to_string(i), cs_handle);
    if (servo_motors[i] == -1 && i < 2 && enable_arm) {
      spdlog::error("Could not locate arm servo motor #{}", i);
      enable_arm = false;
    }
  }
  int blaster_light_handle = -1;
  GimbalValues<int> gimbal_motors{-1, -1};
  if (enable_gimbal) {
    for (size_t i = 0; i < gimbal_motors.size; i++) {
      gimbal_motors[i] = get_handle(gimbal_servo_names[i], cs_handle);
      if (gimbal_motors[i] == -1) {
        spdlog::error("Could not locate gimbal motor #{}", i);
        enable_gimbal = false;
        break;
      }
    }
    blaster_light_handle = get_handle(blaster_light_name, cs_handle);
  }

  GimbalLEDValues<std::vector<int>> gimbal_led_handles;
  if (enable_gimbal) {
    for (size_t i = 0; i < gimbal_led_handles.size; i++) {
      for (size_t j = 0; j < 8; j++) {
        int h = get_handle(gimbal_led_names[i] + std::to_string(j), cs_handle);
        if (h == -1) {
          spdlog::error("Could not locate gimbal led #{}_{}", i, j);
          return -1;
        }
        gimbal_led_handles[i].push_back(h);
      }
    }
  }
  int gripper_handle = get_handle(gripper_name, cs_handle);
  std::string gripper_state = "gripper#" + std::to_string(gripper_handle);
  std::string gripper_target =
      "target_gripper#" + std::to_string(gripper_handle);

  spdlog::info("Gripper signals {} {}", gripper_state, gripper_target);

  const int gyro_handle = get_handle(gyro_name, cs_handle);
  const int accelerometer_handle = get_handle(accelerometer_name, cs_handle);
#if MODEL_VERSION > 1
  const std::string gyro_signal = "signal.value";
  const std::string accelerometer_signal = "signal.value";
  const int imu_handle = get_handle("reference", gyro_handle);
#else
  const std::string gyro_signal = "gyro#" + std::to_string(gyro_handle);
  const std::string accelerometer_signal =
      "accelerometer#" + std::to_string(accelerometer_handle);
  const int imu_handle = get_handle("GyroSensor_reference", cs_handle);
#endif

  _robots.emplace(handle,
                  std::make_unique<CoppeliaSimRobot>(
                      cs_handle, wheel_handles, chassis_led_handles, enable_arm,
                      camera_handle, vision_handle, servo_motors, gimbal_motors,
                      gimbal_led_handles, blaster_light_handle, enable_gripper,
                      gripper_state, gripper_target, imu_handle,
                      accelerometer_handle, accelerometer_signal, gyro_handle,
                      gyro_signal));
  // _interfaces[handle] = std::make_shared<rm::RoboMaster>(&io_context,
  // _robots[handle].get());
  if (remote_api_network.length()) {
    unsigned prefix_len = 0;
    std::string ip = get_ip(remote_api_network, &prefix_len);
    _interfaces.emplace(
        handle, std::make_unique<RoboMaster>(nullptr, _robots[handle].get(),
                                             serial_number, camera_use_udp,
                                             camera_bitrate, ip, prefix_len));
    _interfaces[handle]->spin(true);
  }
  next_robot_handle += 1;
  spdlog::info("Created CoppeliaSim Robot with handle {}", handle);
  return handle;
}

static const std::map<std::string, int> action_handles = {{"move", 1},
                                                          {"move_arm", 2},
                                                          {"move_gimbal", 3},
                                                          {"play_sound", 4},
                                                          {"move_servo", 5}};

std::string action_name(int action_handle) {
  for (auto &[name, i] : action_handles) {
    if (i == action_handle) {
      return name;
    }
  }
  return "";
}

class Plugin : public sim::Plugin {
public:
#if SIM_PROGRAM_VERSION_NB < 40600
  void onModuleHandle(char *customData) {
#else
  void onSimulationBeforeActuation() {
#endif
    auto time_step = simGetSimulationTimeStep();
    for (auto const &[key, val] : _robots) {
      spdlog::debug("Will update RM #{}", key);
      val->do_step(time_step);
      spdlog::debug("Has updated RM #{}", key);
    }
  }

#if SIM_PROGRAM_VERSION_NB < 40600
  void onStart() {
#else
  void onInit() {
#endif
    if (!registerScriptStuff())
      throw std::runtime_error("script stuff initialization failed");

    setExtVersion("Robomaster");
    setBuildDate(BUILD_DATE);
  }

  void onEnd() {}

  void onSimulationAboutToStart() {
    // crobot = new CoppeliaRobot();
    // std::cout << "[RoboMaster plugin] Loaded" << std::endl;
    // robot = std::make_shared<rm::RoboMaster>();
    // crobot->set_wheel_speeds(1.0, -1.0, 1.0, -1.0);
  }

  void onSimulationAboutToEnd() {
    // delete crobot;
    // std::cout << "Should close RM simulation[s] gracefully" << std::endl;
    _interfaces.clear();
    _robots.clear();
    next_robot_handle = 0;
    // std::cout << "Goodbye" << std::endl;
  }

  void create(create_in *in, create_out *out) {
    out->handle =
        add_robot(in->handle, in->serial_number, in->remote_api_network,
                  in->enable_camera, in->camera_use_udp, in->camera_bitrate,
                  in->enable_arm, in->enable_gripper, in->enable_gimbal,
                  in->enable_vision);
  }

  void create_ep(create_ep_in *in, create_ep_out *out) {
    out->handle =
        add_robot(in->handle, in->serial_number, in->remote_api_network, true,
                  false, 1000000, true, true, false, true);
  }

  void create_s1(create_s1_in *in, create_s1_out *out) {
    out->handle =
        add_robot(in->handle, in->serial_number, in->remote_api_network, true,
                  false, 1000000, false, false, true, true);
  }

  // void has_read_accelerometer(has_read_accelerometer_in *in,
  // has_read_accelerometer_out *out) {
  //   if (_robots.count(in->handle)) {
  //     _robots[in->handle]->has_read_accelerometer(in->x, in->y, in->z);
  //   }
  // }
  //
  // void has_read_gyro(has_read_gyro_in *in, has_read_gyro_out *out) {
  //   if (_robots.count(in->handle)) {
  //     _robots[in->handle]->has_read_gyro(in->x, in->y, in->z);
  //   }
  // }

  // void update_orientation(update_orientation_in *in, update_orientation_out
  // *out) {
  //   if (_robots.count(in->handle)) {
  //     _robots[in->handle]->update_orientation(in->alpha, in->beta,
  //     in->gamma);
  //   }
  // }

  void set_target_twist(set_target_twist_in *in, set_target_twist_out *out) {
    if (_robots.count(in->handle)) {
      _robots[in->handle]->chassis.set_target_velocity(
          {.x = in->twist.x, .y = in->twist.y, .theta = in->twist.theta});
    }
  }

  void get_twist(get_twist_in *in, get_twist_out *out) {
    if (_robots.count(in->handle)) {
      auto twist = _robots[in->handle]->chassis.get_twist(Frame::odom);
      out->twist.x = twist.x;
      out->twist.y = twist.y;
      out->twist.theta = twist.theta;
    }
  }

  void get_wheel_speeds(get_wheel_speeds_in *in, get_wheel_speeds_out *out) {
    if (_robots.count(in->handle)) {
      auto speeds = _robots[in->handle]->chassis.wheel_speeds.current;
      out->speeds.front_left = speeds.front_left;
      out->speeds.front_right = speeds.front_right;
      out->speeds.rear_left = speeds.rear_left;
      out->speeds.rear_right = speeds.rear_right;
    }
  }

  void set_target_wheel_speeds(set_target_wheel_speeds_in *in,
                               set_target_wheel_speeds_out *out) {
    if (_robots.count(in->handle)) {
      _robots[in->handle]->chassis.wheel_speeds.set_target(
          {.front_left = in->speeds.front_left,
           .front_right = in->speeds.front_right,
           .rear_left = in->speeds.rear_left,
           .rear_right = in->speeds.rear_right});
    }
  }

  void get_odometry(get_odometry_in *in, get_odometry_out *out) {
    if (_robots.count(in->handle)) {
      auto odom = _robots[in->handle]->chassis.odometry;
      out->odometry = to_cs(odom);
    }
  }

  void get_attitude(get_attitude_in *in, get_attitude_out *out) {
    if (_robots.count(in->handle)) {
      auto value = _robots[in->handle]->chassis.get_attitude();
      out->attitude = to_cs(value);
    }
  }

  void get_imu(get_imu_in *in, get_imu_out *out) {
    if (_robots.count(in->handle)) {
      auto value = _robots[in->handle]->chassis.get_imu();
      out->imu = to_cs(value);
    }
  }

  void move_to(move_to_in *in, move_to_out *out) {
    out->handle = 0;
    if (_robots.count(in->handle)) {
      _robots[in->handle]->move_base(
          {.x = in->pose.x, .y = in->pose.y, .theta = in->pose.theta},
          in->linear_speed, in->angular_speed);
      out->handle = action_handles.at("move");
    }
  }

  void get_action_state(get_action_state_in *in, get_action_state_out *out) {
    if (_robots.count(in->handle)) {
      auto state =
          _robots[in->handle]->get_action_state(action_name(in->action));
      switch (state) {
      case Action::State::failed:
        out->status = "failed";
        break;
      case Action::State::rejected:
        out->status = "rejected";
        break;
      case Action::State::running:
        out->status = "running";
        break;
      case Action::State::succeed:
        out->status = "succeed";
        break;
      case Action::State::undefined:
        out->status = "undefined";
        break;
      case Action::State::started:
        out->status = "started";
        break;
      }
    }
  }

  void set_led_effect(set_led_effect_in *in, set_led_effect_out *out) {
    if (_robots.count(in->handle)) {
      ActiveLED::LedEffect effect;
      if (in->effect == "off") {
        effect = ActiveLED::LedEffect::off;
      } else if (in->effect == "on") {
        effect = ActiveLED::LedEffect::on;
      } else if (in->effect == "breath") {
        effect = ActiveLED::LedEffect::breath;
      } else if (in->effect == "flash") {
        effect = ActiveLED::LedEffect::flash;
      } else if (in->effect == "scrolling") {
        effect = ActiveLED::LedEffect::scrolling;
      } else {
        return;
      }
      _robots[in->handle]->set_led_effect(
          {in->r, in->g, in->b}, LedMask(in->mask),
          CompositeLedMask(in->led_mask), effect, in->period_on, in->period_off,
          in->loop);
    }
  }

  void set_gripper_target(set_gripper_target_in *in,
                          set_gripper_target_out *out) {
    if (_robots.count(in->handle)) {
      Gripper::Status s;
      if (in->state == "pause") {
        s = Gripper::Status::pause;
      } else if (in->state == "open") {
        s = Gripper::Status::open;
      } else if (in->state == "close") {
        s = Gripper::Status::close;
      } else {
        return;
      }
      _robots[in->handle]->gripper.set_target(s, in->power);
    }
  }

  void get_gripper(get_gripper_in *in, get_gripper_out *out) {
    if (_robots.count(in->handle)) {
      auto s = _robots[in->handle]->gripper.get_status();
      if (s == Gripper::Status::pause) {
        out->state = "pause";
      } else if (s == Gripper::Status::open) {
        out->state = "open";
      } else if (s == Gripper::Status::close) {
        out->state = "close";
      } else {
        return;
      }
    }
  }

  void get_arm_position(get_arm_position_in *in, get_arm_position_out *out) {
    if (_robots.count(in->handle)) {
      auto value = _robots[in->handle]->arm.get_position();
      out->position = to_cs(value);
    }
  }

  void move_arm(move_arm_in *in, move_arm_out *out) {
    out->handle = 0;
    if (_robots.count(in->handle)) {
      _robots[in->handle]->move_arm(in->x, in->z, in->absolute);
      out->handle = action_handles.at("move_arm");
    }
  }

  Gimbal::Frame gimbal_frame(std::string name) {
    if (name == "chassis") {
      return Gimbal::Frame::chassis;
    }
    if (name == "fixed") {
      return Gimbal::Frame::fixed;
    }
    return Gimbal::Frame::gimbal;
  }

  void move_gimbal(move_gimbal_in *in, move_gimbal_out *out) {
    out->handle = 0;
    if (_robots.count(in->handle)) {
      _robots[in->handle]->move_gimbal(
          in->yaw, in->pitch, in->yaw_speed, in->pitch_speed,
          gimbal_frame(in->yaw_frame), gimbal_frame(in->pitch_frame));
      out->handle = action_handles.at("move_gimbal");
    }
  }

  void get_gimbal_angles(get_gimbal_angles_in *in, get_gimbal_angles_out *out) {
    if (_robots.count(in->handle)) {
      out->yaw =
          _robots[in->handle]->gimbal.attitude(gimbal_frame(in->yaw_frame)).yaw;
      out->pitch = _robots[in->handle]
                       ->gimbal.attitude(gimbal_frame(in->pitch_frame))
                       .pitch;
    }
  }

  void move_servo(move_servo_in *in, move_servo_out *out) {
    out->handle = 0;
    if (_robots.count(in->handle)) {
      _robots[in->handle]->move_servo(in->servo, in->angle);
      out->handle = action_handles.at("move_servo");
    }
  }

  void set_servo_target_speed(set_servo_target_speed_in *in,
                              set_servo_target_speed_out *out) {
    if (_robots.count(in->handle)) {
      _robots[in->handle]->set_target_servo_speed(in->servo, in->speed);
    }
  }

  void get_servo_angle(get_servo_angle_in *in, get_servo_angle_out *out) {
    if (_robots.count(in->handle)) {
      out->angle = _robots[in->handle]->get_servo_angle(in->servo);
    }
  }

  void set_servo_mode(set_servo_mode_in *in, set_servo_mode_out *out) {
    if (_robots.count(in->handle)) {
      Servo::Mode mode = Servo::Mode::SPEED;
      if (in->mode == "angle") {
        mode = Servo::Mode::ANGLE;
      } else if (in->mode == "speed") {
        mode = Servo::Mode::SPEED;
      } else {
        spdlog::warn("Unknown mode {}", in->mode);
        return;
      }
      _robots[in->handle]->set_servo_mode(in->servo, mode);
    }
  }

  void enable_servo(enable_servo_in *in, enable_servo_out *out) {
    if (_robots.count(in->handle)) {
      _robots[in->handle]->enable_servo(in->servo, in->value);
    }
  }

  void set_gimbal_target_speeds(set_gimbal_target_speeds_in *in,
                                set_gimbal_target_speeds_out *out) {
    if (_robots.count(in->handle)) {
      _robots[in->handle]->gimbal.set_target_speeds(
          {.yaw = in->yaw, .pitch = in->pitch});
    }
  }

  void enable_gimbal(enable_gimbal_in *in, enable_gimbal_out *out) {
    if (_robots.count(in->handle)) {
      _robots[in->handle]->gimbal.enable(in->value);
    }
  }

  void set_blaster_led(set_blaster_led_in *in, set_blaster_led_out *out) {
    if (_robots.count(in->handle)) {
      _robots[in->handle]->set_blaster_led(
          std::clamp(in->intensity, 0.0f, 1.0f), in->intensity > 0);
    }
  }

  void set_mode(set_mode_in *in, set_mode_out *out) {
    if (_robots.count(in->handle)) {
      Robot::Mode mode = Robot::Mode::CHASSIS_LEAD;
      if (in->mode == "free") {
        mode = Robot::Mode::FREE;
      } else if (in->mode == "gimbal_lead") {
        mode = Robot::Mode::GIMBAL_LEAD;
      } else if (in->mode == "chassis_lead") {
        mode = Robot::Mode::CHASSIS_LEAD;
      } else {
        spdlog::warn("Unknown mode {}", in->mode);
        return;
      }
      _robots[in->handle]->set_mode(mode);
    }
  }

  Action::State move_gimbal(float target_yaw, float target_pitch,
                            float yaw_speed, float pitch_speed,
                            Gimbal::Frame yaw_frame, Gimbal::Frame pitch_frame);

  void enable_camera(enable_camera_in *in, enable_camera_out *out) {
    if (_robots.count(in->handle)) {
      if (in->enabled) {
        unsigned width, height;
        if (in->resolution == "720p") {
          width = 1280;
          height = 720;
        } else if (in->resolution == "540p") {
          width = 960;
          height = 540;
        } else {
          width = 640;
          height = 360;
        }
        _robots[in->handle]->start_streaming(width, height);
      } else {
        _robots[in->handle]->stop_streaming();
      }
    }
  }

  void enable_distance_sensor(enable_distance_sensor_in *in,
                              enable_distance_sensor_out *out) {
    if (_robots.count(in->handle)) {
      _robots[in->handle]->enable_tof(in->port, in->sensor_handle);
    }
  }

  void disable_distance_sensor(disable_distance_sensor_in *in,
                               disable_distance_sensor_out *out) {
    if (_robots.count(in->handle)) {
      _robots[in->handle]->tof.set_enable(in->port, false);
    }
  }

  void get_distance_reading(get_distance_reading_in *in,
                            get_distance_reading_out *out) {
    if (_robots.count(in->handle)) {
      auto readings = _robots[in->handle]->tof.readings;
      if (in->port < readings.size()) {
        out->distance = readings[in->port].distance;
      }
    }
  }

  void set_log_level(set_log_level_in *in, set_log_level_out *out) {
    spdlog::set_level(spdlog::level::from_str(in->log_level));
  }

  void get_handles(get_handles_in *in, get_handles_out *out) {
    for (auto &[key, _] : _robots) {
      out->handles.push_back(key);
    }
  }

  void set_vision_class(set_vision_class_in *in, set_vision_class_out *out) {
    if (_robots.count(in->handle)) {
      _robots[in->handle]->set_vision_class(in->name, in->type);
    }
  }

  void enable_vision(enable_vision_in *in, enable_vision_out *out) {
    if (_robots.count(in->handle)) {
      _robots[in->handle]->vision.set_enable(in->mask);
    }
  }

  void get_detected_robots(get_detected_robots_in *in,
                           get_detected_robots_out *out) {
    if (_robots.count(in->handle)) {
      const auto &o = _robots[in->handle]->vision.get_detected_objects();
      for (const auto &r : o.robots) {
        const auto &bb = r.bounding_box;
        out->bounding_boxes.emplace_back(r.uid, bb.x, bb.y, bb.width,
                                         bb.height);
      }
    }
  }

  void get_detected_people(get_detected_people_in *in,
                           get_detected_people_out *out) {
    if (_robots.count(in->handle)) {
      const auto &o = _robots[in->handle]->vision.get_detected_objects();
      for (const auto &p : o.people) {
        const auto &bb = p.bounding_box;
        out->bounding_boxes.emplace_back(p.uid, bb.x, bb.y, bb.width,
                                         bb.height);
      }
    }
  }

  void configure_vision(configure_vision_in *in, configure_vision_out *out) {
    if (_robots.count(in->handle)) {
      _robots[in->handle]->configure_vision(in->min_width, in->min_height,
                                            in->tolerance);
    }
  }
};

#if SIM_PROGRAM_VERSION_NB < 40600
SIM_PLUGIN(PLUGIN_NAME, PLUGIN_VERSION, Plugin)
#else
SIM_PLUGIN(Plugin)
#endif
//
#include "stubsPlusPlus.cpp"
