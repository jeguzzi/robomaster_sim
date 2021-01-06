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

#include "config.h"
#include "plugin.h"
#include "simPlusPlus/Plugin.h"
#include "simPlusPlus/Handle.h"
#include "stubs.h"

#include <spdlog/spdlog.h>

#include "robomaster.hpp"
#include "coppeliasim_robot.hpp"

static int next_robot_handle = 0;
static std::map<int, std::shared_ptr<CoppeliaSimRobot>> _robots;
static std::map<int, std::shared_ptr<rm::RoboMaster>> _interfaces;

// static boost::asio::io_context io_context;

static int add_robot(
    int front_left_wheel, int front_right_wheel, int rear_left_wheel, int rear_right_wheel,
    int front_LED, int left_LED, int rear_LED, int right_LED)
{
  int handle = next_robot_handle;
  next_robot_handle += 1;
  WheelValues<simInt> wheel_handles {front_left_wheel, front_right_wheel, rear_left_wheel, rear_right_wheel};
  LEDValues<simInt> led_handles {front_LED, left_LED, rear_LED, right_LED};
  _robots[handle] = std::make_shared<CoppeliaSimRobot>(wheel_handles, led_handles);
  // _interfaces[handle] = std::make_shared<rm::RoboMaster>(&io_context, _robots[handle].get());
  _interfaces[handle] = std::make_shared<rm::RoboMaster>(nullptr, _robots[handle].get());
  _interfaces[handle]->spin(true);
  return handle;
}


class Plugin : public sim::Plugin
{
public:
    void onStart()
    {
        if(!registerScriptStuff())
            throw std::runtime_error("script stuff initialization failed");

        setExtVersion("Example Plugin Skeleton");
        setBuildDate(BUILD_DATE);
    }

    void onEnd()
    {

    }

    void onSimulationAboutToStart()
    {
      // crobot = new CoppeliaRobot();
      std::cout << "Welcome to the robomaster simulation" << std::endl;
      // robot = std::make_shared<rm::RoboMaster>();
      // crobot->set_wheel_speeds(1.0, -1.0, 1.0, -1.0);

    }

    void onSimulationAboutToEnd()
    {
      // delete crobot;
      std::cout << "Should close RM simulation[s] gracefully" << std::endl;
      _interfaces.clear();
      _robots.clear();
      std::cout << "Goodbye" << std::endl;
    }

    void create(create_in *in, create_out *out)
    {
      out->handle = add_robot(
        in->front_left_wheel, in->front_right_wheel, in->rear_left_wheel, in->rear_right_wheel,
        in->front_LED, in->left_LED, in->rear_LED, in->right_LED
      );

    }

    void has_read_accelerometer(has_read_accelerometer_in *in, has_read_accelerometer_out *out)
    {
      if(_robots.count(in->handle))
      {
        _robots[in->handle]->has_read_accelerometer(in->x, in->y, in->z);
      }
    }

    void has_read_gyro(has_read_gyro_in *in, has_read_gyro_out *out)
    {
      if(_robots.count(in->handle))
      {
        _robots[in->handle]->has_read_gyro(in->x, in->y, in->z);
      }
    }

    void update_orientation(update_orientation_in *in, update_orientation_out *out)
    {
      if(_robots.count(in->handle))
      {
        _robots[in->handle]->update_orientation(in->alpha, in->beta, in->gamma);
      }
    }

    // void onInstancePass(const sim::InstancePassFlags &flags, bool first)
    // {
    //   // spdlog::debug("Pass");
    // }
    //
    void onModuleHandle(char * customData)
    {
      std::string module = customData ? std::string(customData) : "ALL";
      simFloat time_step = simGetSimulationTimeStep();
      simFloat time_ = simGetSimulationTime();
      spdlog::debug("onModuleHandle {} {} ({})", module, time_step, time_);
      for (auto const& [key, val] : _robots)
      {
        spdlog::debug("Will update RM #{}", key);
        val->do_step(time_step);
        spdlog::debug("Has updated RM #{}", key);
      }
    }

    void onModuleHandleInSensingPart(char * customData)
    {
      std::string module = customData ? std::string(customData) : "ALL";
      simFloat time_step = simGetSimulationTimeStep();
      simFloat time_ = simGetSimulationTime();
      spdlog::debug("onModuleHandleInSensingPart {} {} ({})", module, time_step, time_);
    }

};

SIM_PLUGIN(PLUGIN_NAME, PLUGIN_VERSION, Plugin)
#include "stubsPlusPlus.cpp"
