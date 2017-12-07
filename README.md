# RysRos2
A ROS2 software stack for MiniRyś robots.

---
<!-- MarkdownTOC -->

- [Prerequisites](#prerequisites)
	- [Remote management interface \(optional\)](#remote-management-interface-optional)
- [Compilation](#compilation)
- [Running](#running)
	- [Robot](#robot)
	- [Remote](#remote)

<!-- /MarkdownTOC -->

---
## Prerequisites
* Working ROS2 installation (see [link](https://github.com/ros2/ros2/wiki/Installation))
* Working and configured MiniRys robot (TODO: elaborate on specific requirements)

### Remote management interface (optional)
`orocos_kdl` workspace with working `python_orocos_kdl`, e.g. build from [this](https://github.com/mjbogusz/orocos_kinematics_dynamics) source

See step 2. in [compilation](#compilation).

## Compilation
0. Clone this repo - it can be either a self-contained workspace or a sub-workspace of a bigger one. E.g. `git clone https://github.com/GroupOfRobots/RysROS2.git ~/rys-ros2`
1. Source your ROS2 installation, e.g. `source /opt/ros/r2b3/install/setup.bash` or `source ~/ros2/install/setup.zsh` (mind the extension!)
2. (OPTIONAL) Compile the patched `orocos_kdl` workspace and source its setup file, e.g. `source ~/ros2_pykdl/install/setup.zsh`
3. Get inside your workspace (`cd ~/rys-ros2`) and compile the code (`ament build`)

## Running
First you have to source the project's setup file. It will automatically source 'parent' setups, like main ROS2 one or patched `orocos_kdl` one.

```sh
source ~/rys-ros2/install/setup.zsh
```

All executables of this project are run via `ros2 run <package_name> <entrypoint_name>` command.

### Robot
Note: this __WILL NOT__ work without modifications on anything other than MiniRys v3.1 robot.

For running robot-side software you can either run all the nodes separately or use the `rys_launch` package which runs them all from a single entry point.

For all-in-one operation:
```sh
ros2 run rys_launch main
# There is also a multi-threaded equivalent:
ros2 run rys_launch multithread
```

For running nodes separately:
```sh
ros2 run rys_motors_controller main
ros2 run rys_sensor_battery main
ros2 run rys_sensor_imu main
ros2 run rys_sensor_ranges main
ros2 run rys_sensor_temperature main
```

### Remote
This requires the patched `orocos_kdl` package, see [requirements section](#remote-management-interface-optional).

```sh
ros2 run rys_remote main
```
