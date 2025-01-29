# RoboMaster Simulation

This repository contains a library that emulates the firmware and remote API server of [DJI Robomaster S1](https://www.dji.com/robomaster-s1) and [DJI Robomaster EP](https://www.dji.com/robomaster-ep-core) robots.

On-top of this library, we implement a plugin for [CoppeliaSim](https://www.coppeliarobotics.com) that simulates most functionality of the real robots.

We provide also an executable that simulates the robot using a dummy physics, which is mostly useful to test the remote API.

You can use the [official Python client API](https://github.com/dji-sdk/RoboMaster-SDK) or [this fork](https://github.com/jeguzzi/RoboMaster-SDK) --- which fixes some issues --- to control simulated RoboMasters, in the exact same way you would control real RoboMasters.


## Installation

### Dependencies

- [spdlog](https://github.com/gabime/spdlog) (for logging)
- [libavcodec, libavformat, libavutil, libavdevice](https://www.ffmpeg.org) (for compressing videos to H264)
- [boost-system and boost-thread](https://www.boost.org) (for asio)
- [cmake](https://cmake.org) (to build)

#### Linux (Ubuntu)

On Linux, we require a C++-17 compiler.
```bash
$ sudo apt update && sudo apt install libspdlog-dev libboost-system-dev libboost-thread-dev cmake libavcodec-dev libavformat-dev libavutil-dev libavdevice-dev libx264-dev
```

#### MacOs

On MacOs, we require a C++-17 compiler. First install [homebrew](https://brew.sh) if you don't have it already.
```bash
$ brew install spdlog boost cmake ffmpeg
```

#### Windows

On Linux, we require a C++-20 compiler.

*To be completed*.

### CoppeliaSim

To compile and then use the CoppeliaSim plugin you need ... [CoppeliaSim](https://www.coppeliarobotics.com). At the moment, we support CoppeliaSim from v4.0 to v4.9 [latest]. Download the one of the supported version and export the location where you place it:

```bash
export COPPELIASIM_ROOT_DIR=<path to the folder containing the programming subfolder>
```
which on Linux is the root folder that you download, while on MacOs is `/Applications/coppeliaSim.app/Contents/Resources`, if you install the app to the default location.

You also need to install Python>=3.8 and two common dependencies for CoppeliaSim plugins:
- [xsltproc](http://xmlsoft.org/xslt/xsltproc.html). Linux: `sudo apt install xsltproc`. MacOs comes with `xsltproc` installed.
- [xmlschema](https://github.com/sissaschool/xmlschema) (optional): `python3 -m pip install xmlschema`

## Build

#### Linux and MacOs
```bash
$ cd <this repository>
$ mkdir -p build
$ cd build
$ cmake -DCMAKE_BUILD_TYPE=Release ..
$ make install
```

This will build all libraries, executables, plugins and will install the plugin and robot models to coppeliaSim.

## Running

### Dummy simulation

Test if everything is fine by running the dummy simulation
```bash
$ ./test
```

You can have a look at the options to customize it
```
$ ./test --help

Welcome to the robomaster simulation

Usage: ./test <option(s)>
Options:
  --help			Show this help message
  --log_level=<LEVEL>		Log level (default: info)
  --ip=<IP>			Robot ip (default: 0.0.0.0)
  --prefix_len=<LENGTH>			Robot network prefix length (default: 0)
  --serial_number=<SERIAL>	Robot serial number (default: RM0001)
  --udp				Video stream via UDP
  --bitrate=<BITRATE>		Video stream bitrate (default: 200000)
  --armor_hits			Publish armor hits
  --ir_hits			Publish IR hits
  --period=<PERIOD>		Update step [s] (default: 0.05)
```

Then run one of the Python scripts in `examples`
```
$ cd <this repo>/examples
$ python discovery.py
```
which discovers any Robomaster available on the network.

Other scripts showcase various parts of the remote API. For instance
```
$ cd <this repo>/examples
$ python client.py
```
connects to the robot,
```
$ cd <this repo>/examples
$ python camera.py
```
visualizes the video stream (you will see a dummy images),
```
$ cd <this repo>/examples
$ python chassis.py
```
moves the robots while gathering data from the robot base.

### CoppeliaSim simulation

Launch CoppeliaSim. In the model browser, you will find models for Robomaster EP and S1 in `robots > mobile`. Drag one of them to the scene. Press play. You can now interact with the robot through the client library (e.g., to execute the testing scripts).

When the simulation is stopped, you can customize the robot controller (double click on its script), in particular you can set the network interface for the remote API. If you use multiple robots, make sure they use different interfaces, as they all use the same ports (e.g., create a virtual network interface for the second robot).

### Lua interface

Inside CoppeliaSim, we expose a similar interface in lua as the remote API in Python. Take a look [at the list of supported functions](lua_api.md). For example, through Python, you make the robot move forwards like this
```python
  # ep_robot = robot.Robot()
  # ep_robot.initialize(conn_type="sta")
  ep_robot.chassis.drive_speed(0.2, 0, 0)
```
while inside CoppeliaSim, you can send the same command like this
```lua
  simRobomaster.set_target_twist(0, {x=0.2})
  -- 0 identifies one of the robots
```

If you don't need to access the remote API, disable it by setting the network to `''`. Then, you can control as many robots as you want without worrying about networking.
```lua
function sysCall_init()
    handle = simRobomaster.create_s1(sim.getObject("."), "")
end
```

### Distance sensor

We provide a simple model of the [Robomaster distance sensor](https://www.djieducation.com/robomaster.html) implemented in coppeliaSim using a proximity sensor with a single ray.

You find it in the models under `components > sensors > RoboMasterDistanceSensor`.
To use it, drag it to the scene and attach it to a RoboMaster using a [force sensor](https://www.coppeliarobotics.com/helpFiles/en/forceSensors.htm), which is the way to rigidly connect dynamically enabled objects. Then, add the following lines to the `sysCall_init` of the RoboMaster lua script:
```lua
  -- From CoppeliaSim 4.3
  local sensor_handle = sim.getObject('./ToF_sensor')
  -- Before CoppeliaSim 4.3
  -- local sensor_handle = sim.getObjectHandle('ToF_sensor')
  simRobomaster.enable_distance_sensor(handle, 3, sensor_handle)
```

The RoboMaster support up to 4 distance sensors. To enable multiple sensors, loop over each sensor handle:
```lua
  local i=0
  while true do
      -- From CoppeliaSim 4.3
      local sensor_handle = sim.getObject(":/ToF_sensor", {index=i, noError=true})
      -- Before CoppeliaSim 4.3
      -- local sensor_handle = sim.getObjectHandle("ToF_sensor#"..i.."@silentError")
      if sensor_handle < 0 then
          break
      end
      simRobomaster.enable_distance_sensor(handle, i, sensor_handle)
      i = i + 1
  end
```

We include two scenes (`playground_tof_{s1|ep}.ttm`) where 1 (above the s1 camera) or 4 (on the ep chassis) sensors are already attached and enabled.


### Vision

The [real RoboMaster can detect objects](https://robomaster-dev.readthedocs.io/en/latest/python_sdk/robomaster.html#module-robomaster.vision) in the camera stream. In this simulation, it is limited to detecting *people* and *robots* and works as the following:
1. it renders objects using their CoppeliaSim handle as color (see [OpenGL, color coded handles](https://www.coppeliarobotics.com/helpFiles/en/visionSensorPropertiesDialog.htm));
2. it computes the bounding boxes associated to a given object tree;
3. if the bounding box is large enough with respect to the ideal (unobstructed) object size, it is detected.

By default, every "Bill" model is detected as a person and every "RoboMaster" model as a robot.
If you want to associate other models (e.g., if in the scene there are a person named "MyPerson" and a robot named "MyRobot") with people or robots, call
```lua
  simRobomaster.set_vision_class(handle, "MyPerson", simRobomaster.VISION.PERSON)
```
or
```lua
  simRobomaster.set_vision_class(handle, "MyRobot", simRobomaster.VISION.ROBOT)
```

You can configure how large detected bounding boxes must be
```lua
  simRobomaster.configure_vision(handle, 0.5, 0.9, 0.01)
```
passing the minimal relative width (0.5, in this case), minimal relative height (0.9, in this case) and tolerance (1% in this case).

To get the currently detected objects, you can use the Python SDK. Alternatively, from coppeliaSim, you can use the lua API: first, enable the vision module (for example to detect robots)
```lua
  simRobomaster.enable_vision(handle, 1 << simRobomaster.VISION.ROBOT)
```
and then get the detected objects
```lua
  simRobomaster.get_detected_robots(handle)
```
Note that, contrary to the Python SDK, the response from the lua API includes the CoppeliaSim handle of the detected objects.

### Multiple robots

If you want to run multiple instances of the dummy simulation or have multiple robomaster in one CoppeliaSim scene, you need to give to each robot a distinct ip address, as they all use the same ports.

You can use [ip aliasing](https://en.wikipedia.org/wiki/IP_aliasing) to add addresses to one of the available network interfaces. For instance, you may want to use the local loopback interface `lo` to control 2 simulated robots (the same would apply for any other network interface):

1. add a second ip address (we assume that `127.0.0.1/8` is already available)
   ```
     sudo ifconfig lo0 alias 127.0.1.1/8 up
   ```
2. setup the two robots with different addresses and different serial numbers (see below).
3. when finished, you can remove the IP alias with
    ```
      sudo ifconfig lo0 -alias 127.0.1.1
    ```

To setup the robots, for the dummy simulation, launch two simulations
  ```
  ./test --ip=127.0.0.1 --prefix_len=8 --serial_number="RM0"
  ./test --ip=127.0.1.1 --prefix_len=8 --serial_number="RM1"
  ```
and in CoppeliaSim, change the lua scripts of the robots to
```lua
  function sysCall_init()
      handle = simRobomaster.create_s1(sim.getObject("."), "127.0.0.1/8", "RM0")
  end
```
and
```lua
  function sysCall_init()
      handle = simRobomaster.create_s1(sim.getObject("."), "127.0.1.1/8", "RM1")
  end
```

### Troubleshooting

#### simIK

The initial release of Coppelia v4.5 has [a bug](https://forum.coppeliarobotics.com/viewtopic.php?p=38863) 
in the lua script of `simIK`. If you experience errors
using the Robomaster gripper, add the following lines at the begin 
of `simIK.debugGroupIfNeeded` in `simIK.lua`
```lua
function simIK.debugGroupIfNeeded(ikEnv,ikGroup,debugFlags)
  if not _S.ikEnvs then
      _S.ikEnvs={}
  end
  ...
``` 
