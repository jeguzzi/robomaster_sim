// Copyright 2016 Coppelia Robotics AG. All rights reserved.
// marc@coppeliarobotics.com
// www.coppeliarobotics.com
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
// ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// -------------------------------------------------------------------
// Authors:
// Federico Ferri <federico.ferri.it at gmail dot com>
// -------------------------------------------------------------------

#include "plugin.h"
#include "config.h"
#include "simPlusPlus/Handle.h"
#include "simPlusPlus/Plugin.h"
#include "stubs.h"

#include "spdlog/spdlog.h"

#include "coppeliasim_robot.hpp"
#include "robomaster.hpp"

static int next_robot_handle = 0;
static std::map<int, std::unique_ptr<CoppeliaSimRobot>> _robots;
static std::map<int, std::unique_ptr<RoboMaster>> _interfaces;

// static boost::asio::io_context io_context;

static const WheelValues<std::string> wheel_names = {.front_left = "front_left_wheel_joint",
                                                     .front_right = "front_right_wheel_joint",
                                                     .rear_left = "rear_left_wheel_joint",
                                                     .rear_right = "rear_right_wheel_joint"};

static const ChassisLEDValues<std::string> chassis_led_names = {.front = "front_led_link_visual",
                                                                .left = "left_led_link_visual",
                                                                .rear = "rear_led_link_visual",
                                                                .right = "right_led_link_visual"};

static const GimbalLEDValues<std::string> gimbal_led_names = {.top_left = "gimbal_left_led_",
                                                              .top_right = "gimbal_right_led_"};

static const GimbalValues<std::string> gimbal_servo_names = {.yaw = "gimbal_yaw_motor",
                                                             .pitch = "gimbal_pitch_motor"};

static const char servo_name[] = "servo_motor_";
static const char camera_name[] = "Vision_sensor";
static const char blaster_light_name[] = "blaster_light";

static int get_handle(std::string name, std::string suffix) {
  std::string complete_name = name + suffix + "@silentError";
  spdlog::info("Getting handle of {}", complete_name);
  return simGetObjectHandle(complete_name.data());
}

static int add_robot(int coppelia_index, std::string serial_number, bool enable_camera = true,
                     bool camera_use_udp = false, int camera_bitrate = 1000000,
                     bool enable_arm = true, bool enable_gripper = true,
                     bool enable_gimbal = true) {
  int handle = next_robot_handle;
  std::string suffix = "#";
  if (coppelia_index >= 0) {
    suffix += std::to_string(coppelia_index);
  }
  int camera_handle = -1;
  if (enable_camera) {
    camera_handle = get_handle(camera_name, suffix);
    if (camera_handle == -1) {
      spdlog::error("Could not locate camera");
      enable_camera = false;
    }
  }

  WheelValues<simInt> wheel_handles;

  for (size_t i = 0; i < wheel_handles.size; i++) {
    simInt h = get_handle(wheel_names[i], suffix);
    if (h == -1) {
      spdlog::error("Could not locate wheel #{}", i);
      return -1;
    }
    wheel_handles[i] = h;
  }

  ChassisLEDValues<simInt> chassis_led_handles;
  for (size_t i = 0; i < chassis_led_handles.size; i++) {
    simInt h = get_handle(chassis_led_names[i], suffix);
    if (h == -1) {
      spdlog::error("Could not locate led #{}", i);
      return -1;
    }
    chassis_led_handles[i] = h;
  }

  ServoValues<simInt> servo_motors;
  for (size_t i = 0; i < servo_motors.size(); i++) {
    servo_motors[i] = get_handle(servo_name + std::to_string(i), suffix);
    if (servo_motors[i] == -1 && i < 2 && enable_arm) {
      spdlog::error("Could not locate arm servo motor #{}", i);
      enable_arm = false;
    }
  }
  simInt blaster_light_handle = -1;
  GimbalValues<simInt> gimbal_motors{-1, -1};
  if (enable_gimbal) {
    for (size_t i = 0; i < gimbal_motors.size; i++) {
      gimbal_motors[i] = get_handle(gimbal_servo_names[i], suffix);
      if (gimbal_motors[i] == -1) {
        spdlog::error("Could not locate gimbal motor #{}", i);
        enable_gimbal = false;
        break;
      }
    }
    blaster_light_handle = get_handle(blaster_light_name, suffix);
  }

  GimbalLEDValues<std::vector<simInt>> gimbal_led_handles;
  if (enable_gimbal) {
    for (size_t i = 0; i < gimbal_led_handles.size; i++) {
      for (size_t j = 0; j < 8; j++) {
        simInt h = get_handle(gimbal_led_names[i] + std::to_string(j), suffix);
        if (h == -1) {
          spdlog::error("Could not locate gimbal led #{}_{}", i, j);
          return -1;
        }
        gimbal_led_handles[i].push_back(h);
      }
    }
  }
  std::string gripper_state = "gripper" + suffix;
  std::string gripper_target = "target_gripper" + suffix;

  spdlog::info("Gripper signals {} {}", gripper_state, gripper_target);

  _robots.emplace(handle, std::make_unique<CoppeliaSimRobot>(
                              wheel_handles, chassis_led_handles, enable_arm, camera_handle,
                              servo_motors, gimbal_motors, gimbal_led_handles, blaster_light_handle,
                              enable_gripper, gripper_state, gripper_target));
  // _interfaces[handle] = std::make_shared<rm::RoboMaster>(&io_context, _robots[handle].get());
  _interfaces.emplace(handle,
                      std::make_unique<RoboMaster>(nullptr, _robots[handle].get(), serial_number,
                                                   camera_use_udp, camera_bitrate));
  _interfaces[handle]->spin(true);
  next_robot_handle += 1;
  return handle;
}

class Plugin : public sim::Plugin {
 public:
  void onStart() {
    if (!registerScriptStuff())
      throw std::runtime_error("script stuff initialization failed");

    setExtVersion("Example Plugin Skeleton");
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
    // std::cout << "Goodbye" << std::endl;
  }

  void create(create_in *in, create_out *out) {
    out->handle =
        add_robot(in->index, in->serial_number, in->enable_camera, in->camera_use_udp,
                  in->camera_bitrate, in->enable_arm, in->enable_gripper, in->enable_gimbal);
  }

  void has_read_accelerometer(has_read_accelerometer_in *in, has_read_accelerometer_out *out) {
    if (_robots.count(in->handle)) {
      _robots[in->handle]->has_read_accelerometer(in->x, in->y, in->z);
    }
  }

  void has_read_gyro(has_read_gyro_in *in, has_read_gyro_out *out) {
    if (_robots.count(in->handle)) {
      _robots[in->handle]->has_read_gyro(in->x, in->y, in->z);
    }
  }

  void update_orientation(update_orientation_in *in, update_orientation_out *out) {
    if (_robots.count(in->handle)) {
      _robots[in->handle]->update_orientation(in->alpha, in->beta, in->gamma);
    }
  }

  // void onInstancePass(const sim::InstancePassFlags &flags, bool first)
  // {
  //   // spdlog::debug("Pass");
  // }
  //
  void onModuleHandle(char *customData) {
    std::string module = customData ? std::string(customData) : "ALL";
    simFloat time_step = simGetSimulationTimeStep();
    simFloat time_ = simGetSimulationTime();
    spdlog::debug("onModuleHandle {} {} ({})", module, time_step, time_);
    for (auto const &[key, val] : _robots) {
      spdlog::debug("Will update RM #{}", key);
      val->do_step(time_step);
      spdlog::debug("Has updated RM #{}", key);
    }
  }

  void onModuleHandleInSensingPart(char *customData) {
    std::string module = customData ? std::string(customData) : "ALL";
    simFloat time_step = simGetSimulationTimeStep();
    simFloat time_ = simGetSimulationTime();
    spdlog::debug("onModuleHandleInSensingPart {} {} ({})", module, time_step, time_);
  }
};

SIM_PLUGIN(PLUGIN_NAME, PLUGIN_VERSION, Plugin)
#include "stubsPlusPlus.cpp"
