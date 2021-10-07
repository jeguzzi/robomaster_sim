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
$ sudo apt update && sudo apt install libspdlog-dev boost cmake libavcodec-dev libavformat-dev libavutil-dev libavdevice-dev
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

To compile and then use the CoppeliaSim plugin you need ... [CoppeliaSim](https://www.coppeliarobotics.com).
Download the latest release. Export the location where you place it.

```bash
export COPPELIASIM_ROOT_DIR=<path to the folder containing the programming subfolder>
```
which on Linux is the root folder that you download, while on MacOs is `/Applications/coppeliaSim.app/Contents/Resources`, if you install the app to the default location.

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
which discovers any Robomaster avaible on the network.

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
    local index = sim.getNameSuffix(nil)
    handle = simRobomaster.create_s1(index, "")
end
```
