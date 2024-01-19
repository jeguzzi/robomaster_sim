
## Structures
| struct |
|----|
| [CS_Twist2D](#CS_Twist2D) |
| [CS_Pose2D](#CS_Pose2D) |
| [CS_WheelFloatValues](#CS_WheelFloatValues) |
| [CS_Odometry](#CS_Odometry) |
| [CS_Vector3](#CS_Vector3) |
| [CS_IMU](#CS_IMU) |
| [CS_Attitude](#CS_Attitude) |
| [CS_BoundingBox](#CS_BoundingBox) |

## Functions
| generic functions                                                 |
| ----------------------------------------------------------------- |
| [simRobomaster.create](#create)                                   |
| [simRobomaster.set_target_twist](#set_target_twist)               |
| [simRobomaster.get_twist](#get_twist)                             |
| [simRobomaster.get_wheel_speeds](#get_wheel_speeds)               |
| [simRobomaster.set_target_wheel_speeds](#set_target_wheel_speeds) |
| [simRobomaster.get_odometry](#get_odometry)                       |
| [simRobomaster.get_imu](#get_imu)                                 |
| [simRobomaster.get_attitude](#get_attitude)                       |
| [simRobomaster.move_to](#move_to)                                 |
| [simRobomaster.get_action_state](#get_action_state)               |
| [simRobomaster.set_led_effect](#set_led_effect)                   |
| [simRobomaster.enable_camera](#enable_camera)                     |
| [simRobomaster.enable_distance_sensor](#enable_distance_sensor)                     |
| [simRobomaster.enable_disable_distance_sensor](#enable_disable_distance_sensor)                     |
| [simRobomaster.get_distance_reading](#get_distance_reading)                     |
| [simRobomaster.enable_vision](#enable_vision)                     |
| [simRobomaster.configure_vision](#configure_vision)                     |
| [simRobomaster.set_vision_class](#set_vision_class)                     |
| [simRobomaster.get_detected_robots](#get_detected_robots)                     |
| [simRobomaster.get_detected_people](#get_detected_people)                     |
| [simRobomaster.set_log_level](#set_log_level)                     |
| [simRobomaster.get_handles](#get_handles)                         |
| [simRobomaster.wait_for_completed](#wait_for_completed)            |

| s1-specific functions |
|----|
| [simRobomaster.create_s1](#create_s1) |
| [simRobomaster.move_gimbal](#move_gimbal) |
| [simRobomaster.get_gimbal_angles](#get_gimbal_angles) |
| [simRobomaster.set_gimbal_target_speeds](#set_gimbal_target_speeds) |
| [simRobomaster.enable_gimbal](#enable_gimbal) |
| [simRobomaster.set_blaster_led](#set_blaster_led) |
| [simRobomaster.set_mode](#set_mode) |

| ep-specific functions                                           |
| --------------------------------------------------------------- |
| [simRobomaster.create_ep](#create_ep)                           |
| [simRobomaster.set_gripper_target](#set_gripper_target)         |
| [simRobomaster.get_gripper](#get_gripper)                       |
| [simRobomaster.get_arm_position](#get_arm_position)             |
| [simRobomaster.move_arm](#move_arm)                             |
| [simRobomaster.get_servo_angle](#get_servo_angle)               |
| [simRobomaster.move_servo](#move_servo)                         |
| [simRobomaster.set_servo_target_speed](#set_servo_target_speed) |
| [simRobomaster.set_servo_mode](#set_servo_mode)                 |
| [simRobomaster.enable_servo](#enable_servo)                     |
| [simRobomaster.wait_for_gripper](#wait_for_gripper)                     |
| [simRobomaster.open_gripper](#open_gripper)                     |
| [simRobomaster.close_gripper](#close_gripper) |

#### CS_Twist2D

```C++
CS_Twist2D = {float x= 0.0, float y= 0.0, float theta= 0.0}
```

*fields*
  - **x** Longitudinal linear speed [m/s], positive towards forward
  - **y** Transversal linear speed [m/s], positive towards left
  - **theta** Angular speed [rad/s] around the vertical axis


#### CS_Pose2D

```C++
CS_Pose2D = {float x= 0.0, float y= 0.0, float theta= 0.0}
```

*fields*
  - **x** x-component of position [m]
  - **y** y-component of position [m]
  - **theta** orintation [rad]


#### CS_WheelFloatValues
A structure with one number per wheel
```C++
CS_WheelFloatValues = {float front_left= 0.0, float front_right= 0.0, float rear_left= 0.0, float rear_right= 0.0}
```

*fields*
  - **front_left** Front left wheel value
  - **front_right** Front right wheel value
  - **rear_left** Rear left wheel value
  - **rear_right** Rear right wheel value


#### CS_Odometry
Odometry (pose and twist)
```C++
CS_Odometry = {CS_Pose2D pose, CS_Twist2D twist}
```

*fields*
  - **pose** The pose
  - **twist** The twist


#### CS_Vector3
A 3D vector
```C++
CS_Vector3 = {float x, float y, float z}
```

*fields*
  - **x** x-component
  - **y** y-component
  - **z** z-component


#### CS_IMU
IMU state
```C++
CS_IMU = {CS_Vector3 angular_velocity, CS_Vector3 acceleration}
```

*fields*
  - **angular_velocity** Angular velocity
  - **acceleration** Acceleration


#### CS_Attitude
Attitude (Tait–Bryan)
```C++
CS_Attitude = {float yaw, float pitch, float roll}
```

*fields*
  - **yaw** Yaw [rad]
  - **pitch** Pitch [rad]
  - **roll** Roll [rad]


#### CS_BoundingBox
Bounding box in image frame of a detected object. 
```C++
CS_BoundingBox = {float handle, float x, float y, float width, float height}
```

*fields*
  - **handle** The handle of the object.
  - **x** Horizontal position in the image, normalized between 0 (left) and 1 (right)
  - **y** Vertical position in the image, normalized between 0 (bottom) and 1 (top)
  - **width** Width as fraction of the frame width
  - **height** Height as fraction of the frame height

#### create
Instantiate a RoboMaster controller
```C++
int handle = simRobomaster.create(int handle, string remote_api_network="", string serial_number="", bool camera_use_udp=false, int camera_bitrate=1000000, bool enable_camera=true, bool enable_gripper=true, bool enable_arm=true, bool enable_gimbal=true, bool enable_vision=true)
```

*parameters*
  - **handle** The handle of the CoppeliaSim robot.
  - **remote_api_network** The address that the remote API should use `"<ip>/<subnet size in bits>"`. E.g., use `"127.0.0.1/24"` for local network, `"/0"` to bind to any network interface. Leave empty to disable the remote API.
  - **serial_number** The robot serial number
  - **camera_use_udp** Make the camera use UDP for streaming
  - **camera_bitrate** The target bitrate for the camera stream
  - **enable_camera** Enable the camera module
  - **enable_gripper** Enable the camera module
  - **enable_arm** Enable the robotic arm module
  - **enable_gimbal** Enable the gimbal module
  - **enable_vision** Enable the vision module

*return*
  - **handle** An handle that identifies the RoboMaster controller


#### create_ep
Instantiate a RoboMaster controller with the default configuration for EP:
          `enable_camera=true`, `camera_use_udp=false`, `camera_bitrate=1000000`, `enable_arm=true`, `enable_gripper=true`, `enable_gimbal=false`, `enable_vision=true`

```C++
int handle = simRobomaster.create_ep(int handle, string remote_api_network="", string serial_number="")
```

*parameters*
  - **handle** The handle of the CoppeliaSim robot.
  - **remote_api_network** The address that the remote API should use `"<ip>/<subnet size in bits>"`. E.g., use `"127.0.0.1/24"` for local network, `"/0"` to bind to any network interface. Leave empty to disable the remote API.
  - **serial_number** The robot serial number

*return*
  - **handle** An handle that identifies the RoboMaster controller


#### create_s1
Instantiate a RoboMaster controller with the default configuration for S1:
        `enable_camera=true`, `camera_use_udp=false`, `camera_bitrate=1000000`, `enable_arm=false`, `enable_gripper=false`, `enable_gimbal=true`, `enable_vision=true`

```C++
int handle = simRobomaster.create_s1(int handle, string remote_api_network="", string serial_number="")
```

*parameters*
  - **handle** The handle of the CoppeliaSim robot.
  - **remote_api_network** The address that the remote API should use `"<ip>/<subnet size in bits>"`. E.g., use `"127.0.0.1/24"` for local network, `"/0"` to bind to any network interface. Leave empty to disable the remote API.
  - **serial_number** The robot serial number

*return*
  - **handle** An handle that identifies the RoboMaster controller


#### set_target_twist
Set the chassis target twist (linear and angular velocity)
```C++
simRobomaster.set_target_twist(int handle, CS_Twist2D twist)
```

*parameters*
  - **handle** The RoboMaster controller handle
  - **twist** The target twist




#### get_twist
Get the chassis current twist estimation (linear and angular velocity)
```C++
CS_Twist2D twist = simRobomaster.get_twist(int handle)
```

*parameters*
  - **handle** The RoboMaster controller handle

*return*
  - **twist** The current twist


#### get_wheel_speeds
Get the current wheel speeds estimation
```C++
CS_WheelFloatValues speeds = simRobomaster.get_wheel_speeds(int handle)
```

*parameters*
  - **handle** The RoboMaster controller handle

*return*
  - **speeds** The current wheel speeds in m/s


#### set_target_wheel_speeds
Set the target wheel speeds
```C++
simRobomaster.set_target_wheel_speeds(int handle, CS_WheelFloatValues speeds)
```

*parameters*
  - **handle** The RoboMaster controller handle
  - **speeds** The target wheel speeds in m/s




#### get_odometry
Get the current odometry state estimation
```C++
CS_Odometry odometry = simRobomaster.get_odometry(int handle)
```

*parameters*
  - **handle** The RoboMaster controller handle

*return*
  - **odometry** The current odometry


#### get_imu
Get the current IMU estimation
```C++
CS_IMU imu = simRobomaster.get_imu(int handle)
```

*parameters*
  - **handle** The RoboMaster controller handle

*return*
  - **imu** The current IMU


#### get_attitude
Get the current attitude estimation
```C++
CS_Attitude attitude = simRobomaster.get_attitude(int handle)
```

*parameters*
  - **handle** The RoboMaster controller handle

*return*
  - **attitude** The current attitude


#### move_to
Make the robot go to a target pose. This will create an possibly long running action.
```C++
int handle = simRobomaster.move_to(int handle, CS_Pose2D pose, float linear_speed=0.5, float angular_speed=1.0)
```

*parameters*
  - **handle** The RoboMaster controller handle
  - **pose** The target pose
  - **linear_speed** The desired linear speed
  - **angular_speed** The angular linear speed

*return*
  - **handle** The action handle


#### get_action_state

```C++
string status = simRobomaster.get_action_state(int handle, int action)
```

*parameters*
  - **handle** The RoboMaster controller handle
  - **action** The action handle

*return*
  - **status** The status of the action, one of `"failed"`, `"rejected"`, `"running"`, `"undefined"`, `"started"`


#### set_led_effect
Apply a LED effect
```C++
simRobomaster.set_led_effect(int handle, float r, float g, float b, string effect="on", int mask=0xFF, int led_mask=0xFF, float period_on=0.5, float period_off=0.5, bool loop=true)
```

*parameters*
  - **handle** The RoboMaster controller handle
  - **r** The red intensity between 0 and 1
  - **g** The green intensity between 0 and 1
  - **b** The blue intensity between 0 and 1
  - **effect** The effect to apply, on of `"on"`, `"off"`, `"breath"`, `"flash"`, `"scrolling"`
  - **mask** A bit-mask to select LED:
                  chassis back (0x1)
                  chassis front (0x2)
                  chassis left (0x4)
                  chassis right (0x8)
                  gimbal left (0x10)
                  gimbal right (0x20)

  - **led_mask** A bit mask to select the individual portions of gimbal LEDs
  - **period_on** The duration of the on-time [ms]
  - **period_off** The duration of the off-time [ms]
  - **loop** Whever the effect should be repeated continously




#### set_gripper_target
Set the gripper target state
```C++
simRobomaster.set_gripper_target(int handle, string state, float power=0.5)
```

*parameters*
  - **handle** The RoboMaster controller handle
  - **state** The target state, one of `"open"`, `"close"`, `"pause"`
  - **power** The desired power level between 0 and 1




#### get_gripper
Get the gripper current state
```C++
string state = simRobomaster.get_gripper(int handle)
```

*parameters*
  - **handle** The RoboMaster controller handle

*return*
  - **state** The current state, one of `"open"`, `"close"`, `"pause"`


#### get_arm_position
Get the robotic arm end effector current position estimation
```C++
CS_Vector3 position = simRobomaster.get_arm_position(int handle)
```

*parameters*
  - **handle** The RoboMaster controller handle

*return*
  - **position** The current position


#### move_arm
Make the robotic arm move the end effect to a target position
```C++
int handle = simRobomaster.move_arm(int handle, float x, float z, bool absolute)
```

*parameters*
  - **handle** The RoboMaster controller handle
  - **x** The x-coordination of the target position [m]
  - **z** The z-coordination of the target position [m]
  - **absolute** Whenver to consider the position absolute with respect to the robot frame, or relative with respect to the current arm position

*return*
  - **handle** The handle of the action


#### enable_camera
Set the camera module state
```C++
simRobomaster.enable_camera(int handle, bool enabled, string resolution="720p")
```

*parameters*
  - **handle** The RoboMaster controller handle
  - **enabled** The camera state
  - **resolution** The camera vertical resolution, one of `"480p"`, `"540p"`, and `"720p"`




#### move_gimbal
Make the gimbal move to a target attitude
```C++
int handle = simRobomaster.move_gimbal(int handle, float yaw, float pitch, float yaw_speed, float pitch_speed, string yaw_frame="chassis", string pitch_frame="fixed")
```

*parameters*
  - **handle** The RoboMaster controller handle
  - **yaw** The target yaw [rad]
  - **pitch** The target pitch [rad]
  - **yaw_speed** The desired yaw speed [rad/s]
  - **pitch_speed** The desired pitch speed [rad/s]
  - **yaw_frame** The target yaw frame, one of `"chassis"`, `"fixed"`, or `"gimbal"`
  - **pitch_frame** The target pitch frame, one of `"fixed"` or `"gimbal"`

*return*
  - **handle** The action handle


#### get_gimbal_angles
Get the current gimbal state estimation
```C++
float yaw, float pitch = simRobomaster.get_gimbal_angles(int handle, string yaw_frame="chassis", string pitch_frame="fixed")
```

*parameters*
  - **handle** The RoboMaster controller handle
  - **yaw_frame** The yaw frame, one of `"chassis"`, `"fixed"`, or `"gimbal"`
  - **pitch_frame** The pitch frame, one of `"fixed"` or `"gimbal"`

*return*
  - **yaw** The current yaw [rad]
  - **pitch** The current pitch [rad]


#### set_gimbal_target_speeds
Set the gimbal target angular speeds
```C++
simRobomaster.set_gimbal_target_speeds(int handle, float yaw, float pitch)
```

*parameters*
  - **handle** The RoboMaster controller handle
  - **yaw** The target yaw speed [rad/s]
  - **pitch** The target pitch speed [rad/s]




#### get_servo_angle
Get the current servo angle estimation
```C++
float angle = simRobomaster.get_servo_angle(int handle, int servo)
```

*parameters*
  - **handle** The RoboMaster controller handle
  - **servo** The index of the servo (0, 1, 2)

*return*
  - **angle** The current servo angle [rad]


#### enable_gimbal
Set the gimbal state
```C++
simRobomaster.enable_gimbal(int handle, bool value)
```

*parameters*
  - **handle** The RoboMaster controller handle
  - **value** Set to true to enable the gimbal motors, set to false to disable them




#### set_blaster_led
Set the intensity of the blaster LED
```C++
simRobomaster.set_blaster_led(int handle, float intensity)
```

*parameters*
  - **handle** The RoboMaster controller handle
  - **intensity** The intensity between 0 (off) and 1




#### move_servo
Make the robot move one of the servo motors to a target angle
```C++
int handle = simRobomaster.move_servo(int handle, int servo, float angle)
```

*parameters*
  - **handle** The RoboMaster controller handle
  - **servo** The index of the servo (0, 1, 2)
  - **angle** The target angle [rad]

*return*
  - **handle** The handle of the action


#### set_servo_target_speed
Set the target speed of one the servo motors
```C++
simRobomaster.set_servo_target_speed(int handle, int servo, float speed)
```

*parameters*
  - **handle** The RoboMaster controller handle
  - **servo** The index of the servo (0, 1, 2)
  - **speed** The target speed [rad/s]




#### set_servo_mode
Set the control mode of one the servo motors
```C++
simRobomaster.set_servo_mode(int handle, int servo, string mode)
```

*parameters*
  - **handle** The RoboMaster controller handle
  - **servo** The index of the servo (0, 1, 2)
  - **mode** The control mode, one of `"angle"` and `"speed"`




#### enable_servo
Set the state of one of the servo motors
```C++
simRobomaster.enable_servo(int handle, int servo, bool value)
```

*parameters*
  - **handle** The RoboMaster controller handle
  - **servo** The index of the servo (0, 1, 2)
  - **value** Set to true to enable motor control or to false to disable it




#### set_mode
Set how the robot controller couples chassis and gimbal
```C++
simRobomaster.set_mode(int handle, string mode)
```

*parameters*
  - **handle** The RoboMaster controller handle
  - **mode** On of `"free"` (uncoupled), `"gimbal_lead"` (the chassis follows the gimbal), `"chassis_lead"` (the gimbal follows the chassis)




#### set_log_level
Set the log level of all Robomaster controllers. Log are displayed on the console.
```C++
simRobomaster.set_log_level(string log_level)
```

*parameters*
  - **log_level** The log level, one of `"debug"`,  `"info"`, `"warning"`, `"error"`




#### get_handles
Get the handles of all active RoboMaster controllers
```C++
table[int] handles = simRobomaster.get_handles()
```

*return*
  - **handles** The list of handles


#### wait_for_completed
Block until the action is completed. To be called from a co-routine.
```C++
simRobomaster.wait_for_completed(int robot_handle, int action_handle)
```

*parameters*
  - **robot_handle** The RoboMaster controller handle
  - **action_handle** The action handle

#### wait_for_gripper
Block until the gripper reaches the target state. To be called from a co-routine.
```C++
simRobomaster.wait_for_gripper(int robot_handle, string target_state)
```

*parameters*
  - **robot_handle** The RoboMaster controller handle
  - **target_state** The action handle

#### open_gripper
Make the gripper open the gripper. To be called from a co-routine.
```C++
simRobomaster.open_gripper(int robot_handle, bool wait=true)
```

*parameters*
  - **robot_handle** The RoboMaster controller handle
  - **wait** Whenever the call should block until the robot finishes opening the gripper

#### close_gripper
Make the gripper close the gripper. To be called from a co-routine.
```C++
simRobomaster.close_gripper(int robot_handle, bool wait=true)
```

*parameters*
  - **robot_handle** The RoboMaster controller handle
  - **wait** Whenever the call should block until the robot finishes closing the gripper


#### enable_distance_sensor
Connect a distance sensor to the robot and enable it.
```C++
simRobomaster.enable_distance_sensor(int handle, int port, int sensor_handle)
```

*parameters*
  - **handle** The RoboMaster controller handle
  - **port** The port at which the sensor is connected (from 0 to 3)
  - **sensor_handle** The handle of the distance sensor


#### disable_distance_sensor
Disable a distance sensor.
```C++
simRobomaster.disable_distance_sensor(int handle, int port)
```

*parameters*
  - **handle** The RoboMaster controller handle
  - **port** The port at which the sensor is connected (from 0 to 3)

#### get_distance_reading
Disable a distance sensor.
```C++
float simRobomaster.get_distance_reading(int handle, int port)
```

*parameters*
  - **handle** The RoboMaster controller handle
  - **port** The port at which the sensor is connected (from 0 to 3)

*return*
  - **distance** The distance reading in meters: negative if no object is in range, 0 if the sensor is not enabled, positive if an object was detected.


#### enable_vision
Enable/disable vision. It replicates the SDK API.
```C++
simRobomaster.enable_vision(int handle, int mask)
```

*parameters*
  - **handle** The RoboMaster controller handle
  - **mask** A bit mask: set to 0 to disable vision, to 2 (1<<1) to detect people, or to 128 (1 << 7) to detect robots.


#### configure_vision
Set the vision tolerance, i.e., how much of the object has to be visible to be detected.
```C++
simRobomaster.configure_vision(int handle, float min_width, float min_height, float tolerance)
```

*parameters*
  - **handle** The RoboMaster controller handle
  - **min_width** Minimal visible horizontal portion of the object to be detectable.
  - **min_height** Minimal visible vertical portion of the object to be detectable.
  - **tolerance** Tolerance for the object to be detectable (between 0 and 1)


#### set_vision_class
Set the vision class associated to a model
```C++
simRobomaster.set_vision_class(int handle, string name, int type)
```

*parameters*
  - **handle** The RoboMaster controller handle
  - **name** The name of the model (without the leading "/")
  - **type** The vision type associated to the model (enum of type VISION)


#### get_detected_robots
Disable a distance sensor.
```C++
[CS_BoundingBox] simRobomaster.get_detected_robots(int handle)
```

*parameters*
  - **handle** The RoboMaster controller handle

*return*
  - **bounding_boxes** A list with bounding boxes and handles of the currently detected robots.

#### get_detected_people
Disable a distance sensor.
```C++
[CS_BoundingBox] simRobomaster.get_detected_people(int handle)
```

*parameters*
  - **handle** The RoboMaster controller handle

*return*
  - **bounding_boxes** A list with bounding boxes and handles of the currently detected people.
