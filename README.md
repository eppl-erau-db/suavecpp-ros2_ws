# SUAVE Drone Control Workspace (ROS 2 Humble + PX4)

Minimal offboard flight stack: `suave_main` connects to PX4 over MAVSDK, feeds RealSense + RTAB-Map visual-inertial odometry into PX4 as external vision, and flies from console commands.

```
src/suave_launch/
├── launch/suave_slam.py           RTAB-Map RGB-D odometry + IMU filter (started by suave_main)
└── src/
    ├── suave_main.cpp             entry point, Ctrl-C = land and exit
    ├── controllers/               SuaveFlightController (console commands), ControllerMacros
    ├── mavutil/                   PX4 connection (MavUtil), Drone wrapper, TelemetryProperty
    ├── vio/VIOBridge              RTAB-Map /odom -> PX4 Mocap (vision position + odometry)
    ├── ros/RosNodeSpinner         multithreaded executor task
    ├── common/                    task / process helpers
    └── util/                      logging, Result, Timer
```

## Setup (companion computer, Ubuntu 22.04)
```
sudo apt install git -y
mkdir ~/Dev && cd ~/Dev
git clone https://github.com/Destiny-Dragon-604/dronecpp-ros2_ws.git
cd dronecpp-ros2_ws/setup
source setup.bash      # ROS 2 Humble, RealSense, RTAB-Map, MAVSDK 1.4.16, FTDI id; reboot after
```

## Build
```
ros                    # alias: source /opt/ros/humble/setup.bash
cd ~/Dev/dronecpp-ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source ./install/setup.bash
```

## Fly
The connection is chosen by `SUAVE_MAVLINK_SERIAL_ID`:
- unset or `SITL` -> `udp://:14540`, camera/VIO stack skipped (PX4 SITL has its own estimate)
- FTDI id (set by `setup/find_ftdi_id.pl`) -> `serial:///dev/serial/by-id/<id>:57600`, camera/VIO stack started

**Simulation:** in one terminal
```
cd ~/Dev/PX4-Autopilot && make px4_sitl gz_x500
```
then in another
```
ros && source ~/Dev/dronecpp-ros2_ws/install/setup.bash
ros2 run suave_launch suave_main
```

**Real drone:** put the flight controller in **manual** mode (PX4 will refuse offboard otherwise), run `ros2 run suave_launch suave_main`, walk the drone around to build the initial RTAB-Map map, press enter, then command it.

### Commands (NED, metres, down < 0 is up)
| command | action |
|---|---|
| `takeoff [alt]` | arm, enter offboard, climb to `alt` above the takeoff point (default 1.75, max 5) |
| `move dn de dd` | step relative to the current position (max 5 m) |
| `goto n e d` | fly to a point relative to the takeoff point (max 5 m) |
| `hold` | hold the current position |
| `land` | descend at 0.5 m/s, disarm on touchdown (falls back to PX4 land after 30 s) |
| `status` | position, heading, armed / in-air / offboard |
| `exit` | land if airborne, stop camera/VIO, quit |

Ctrl-C at any time lands and exits.

To watch the map over ssh (`ssh -YC suave@<ip>`), run the SLAM launch by hand with `viz:=true`.

## Troubleshooting: `offboard start failed: Command Denied`
1. The drone must be in manual mode when the script starts.
2. The RealSense must be on a USB 3 port/cable (blue port); check with `realsense-viewer`.
3. If the RealSense works, run RTAB-Map on its own and confirm it opens:
   ```
   ros2 launch realsense2_camera rs_launch.py enable_gyro:=true enable_accel:=true unite_imu_method:=1 enable_infra1:=true enable_infra2:=true enable_sync:=true
   ros2 param set /camera/camera depth_module.emitter_enabled 0
   ros2 launch suave_launch suave_slam.py viz:=true
   ```
4. Bad FTDI link: check `ls /dev/serial/by-id/` matches `$SUAVE_MAVLINK_SERIAL_ID`, reseat the board and reboot. After swapping boards:
   ```
   echo 'export SUAVE_MAVLINK_SERIAL_ID=usb-FTDI_FT232R_USB_UART_<NEW_ID>-if00-port0' >> ~/.bashrc
   source ~/.bashrc
   ```
