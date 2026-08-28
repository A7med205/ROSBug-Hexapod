[Project home](../README.md) · [Documentation index](README.md)

# Simulation

## ROS 2 and RViz

Build the package in a ROS 2 Humble environment:

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Launch Gazebo Sim with RViz in the first terminal:

```bash
ros2 launch hexapod_sim robot.launch.py sim:=true rviz:=true use_sim_time:=true
```

In a second sourced terminal, run the shared keyboard controller:

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run hexapod_sim sim_interface.py
```

The adapter parameters can be overridden through ROS arguments:

```bash
ros2 run hexapod_sim sim_interface.py --ros-args \
  -p action_name:=/joint_trajectory_controller/follow_joint_trajectory \
  -p wait_timeout:=10.0
```

| Launch argument | Default | Effect |
| --- | ---: | --- |
| `sim` | `true` | Starts Gazebo Sim, spawns the robot, bridge, and ros2_control controllers |
| `rviz` | `false` | Starts RViz with `rviz/config.rviz` |
| `use_sim_time` | `true` | Selects the Gazebo `/clock` for ROS nodes |

Use `ros2 launch hexapod_sim robot.launch.py --show-args` for these and the
version-dependent arguments forwarded by the Gazebo launch file.

Common launch variants:

```bash
# Gazebo Sim only (all defaults)
ros2 launch hexapod_sim robot.launch.py

# Gazebo Sim and RViz
ros2 launch hexapod_sim robot.launch.py rviz:=true

# RViz-only model inspection; no trajectory action server is created
ros2 launch hexapod_sim robot.launch.py sim:=false rviz:=true use_sim_time:=false
```

Do not run `sim_interface.py` with the RViz-only variant unless another node
provides the configured trajectory action server.

## Docker

The Docker image uses `osrf/ros:humble-desktop-full`, copies the checkout,
installs the dependencies declared by `hexapod_sim`, and builds the ROS
workspace. From the repository root, build and run Gazebo, RViz, and the
interactive keyboard interface with:

```bash
./run_sim_docker.sh
```

The script forwards the host X11 display, host networking, the terminal, and
`/dev/dri` when available. It rebuilds with Docker's layer cache before each
run. The resulting desktop image is approximately 4 GB. To reuse an existing
image or omit RViz:

```bash
./run_sim_docker.sh --no-build
./run_sim_docker.sh --no-rviz
```

Run `./run_sim_docker.sh --help` for image-name and environment overrides.
Closing the keyboard interface also stops the launch process and removes the
container. The host must provide Docker, `xhost`, an X11-compatible `DISPLAY`,
and access to the Docker daemon.

## Simulation-only parameters

| Parameter | Value |
| --- | ---: |
| Gazebo spawn pose | `(x, y, z) = (0, 0, 0.25)` m |
| ros2_control update rate | 100 Hz |
| URDF revolute limits | `[-π, +π]` rad |
| URDF joint velocity limit | `2π` rad/s |
| URDF joint effort limit | 1000 |
| ros2_control initial `j1/j2/j3` | `0, -0.955, 2.216` rad |
| Tip friction `mu1`, `mu2` | 1.2, 1.2 |
| Tip contact `kp`, `kd` | `1e6`, 100 |
| Tip contact `minDepth`, `maxVel` | 0.001 m, 0.1 m/s |

The controller uses position commands and position/velocity state interfaces.
The shared neutral IK pose is approximately `(0, -0.787, 2.140)` rad per leg.
