# Mobile Robot SLAM + Gaussian Splatting

> **Work in progress.** Currently in Phase 1: the robot is described in URDF and drives in Gazebo Harmonic. There is no hardware integration yet.

An autonomous differential-drive robot built on an ActivMedia AmigoBot chassis, running ROS 2 Jazzy. The goal is fully autonomous room capture:

1. 2D lidar SLAM provides the robot's poses.
2. The robot explores the room on its own.
3. Its RGB-D data is used to build a **3D Gaussian Splatting digital twin**.

**Research idea:** uncertainty-driven exploration, where the geometric map tells the robot where the twin still lacks good views. The point is autonomous, metric, repeatable capture, compared against a handheld video + COLMAP + 3DGS baseline.

## Status

| Phase | Scope | State |
|---|---|---|
| 1. Base | URDF, Gazebo sim, teleop, `ros2_control`, firmware, odometry calibration (UMBmark) | 🚧 In progress |
| 2. SLAM and autonomy | `slam_toolbox`, `robot_localization`, Nav2, frontier exploration | ⏳ Not started |
| 3. Gaussian Splatting | Offline (rosbags → splatfacto / gsplat), then online with lidar pose priors | ⏳ Not started |
| 4. Contribution | Uncertainty-driven exploration against the frontier baseline | ⏳ Not started |



## Hardware

| Part | Choice |
|---|---|
| Chassis | ActivMedia AmigoBot (~33 × 28 × 13 cm, 3.6 kg stock, 1 kg payload) |
| Motors | 2× Pittman GM8212D142, 12 V, 19.5:1, 39,000 encoder counts per wheel revolution |
| Compute | Raspberry Pi 5 (8 GB) + NVMe SSD. 3DGS training runs on a desktop GPU. |
| MCU | ESP32 (planned) |
| Motor driver | XY-160D (planned) |
| Lidar | Slamtec RPLIDAR C1 |
| Depth camera | Intel RealSense D435i |
| Battery | 3S Li-ion with BMS |

See `BOM.txt` for the parts list.

## Repository layout

```
ros2_ws/src/mobile_robot/
├── description/
│   ├── robot.urdf.xacro       # top-level robot description
│   ├── robot_core.xacro       # base_link, chassis, wheels, caster
│   ├── inertial_macros.xacro  # inertia helpers (box, cylinder, sphere)
│   └── gazebo_control.xacro   # Gazebo DiffDrive + JointStatePublisher plugins
├── launch/
│   ├── rsp.launch.py          # robot_state_publisher
│   └── launch_sim.launch.py   # Gazebo + spawn + ROS–Gazebo bridge
├── config/
│   ├── gz_bridge.yaml         # topics bridged between Gazebo and ROS
│   ├── view_bot.rviz          # RViz: inspect the model
│   └── drive_bot.rviz         # RViz: drive it in sim
└── worlds/
    ├── empty.world
    └── obstacles.world
```

## Setup

Tested on Ubuntu 24.04 (including WSL2) with ROS 2 Jazzy.

```bash
sudo apt install ros-jazzy-xacro ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher-gui ros-jazzy-rviz2 \
  ros-jazzy-ros-gz ros-jazzy-teleop-twist-keyboard

cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Usage

**View the model in RViz**
```bash
ros2 launch mobile_robot rsp.launch.py
ros2 run joint_state_publisher_gui joint_state_publisher_gui
rviz2 -d src/mobile_robot/config/view_bot.rviz
```

**Simulate in Gazebo and drive with the keyboard**
```bash
ros2 launch mobile_robot launch_sim.launch.py   # add world:=<path to .world> for another world
ros2 run teleop_twist_keyboard teleop_twist_keyboard
rviz2 -d src/mobile_robot/config/drive_bot.rviz
```

### TF tree (current)
```
odom → base_link → chassis → caster_wheel
                 → left_wheel
                 → right_wheel
```
`odom → base_link` comes from Gazebo's DiffDrive plugin, through the bridge. The target tree adds `laser` and `camera_link`.

## Known placeholders

These values are nominal and **must be measured** on the real robot:
- Wheel radius: 0.05 m (nominal 10 cm wheel)
- Track width: 0.28 m in the URDF. The original firmware suggests about 0.25 m.
- Wheel and caster masses (0.1 kg each) and caster position
- Motor electrical constants and the encoder pinout

## Roadmap

1. Add the lidar and camera to the URDF and the Gazebo sensors.
2. Switch from the Gazebo DiffDrive plugin to `ros2_control` (`gz_ros2_control` in sim, then a real hardware interface).
3. Write the firmware, then calibrate odometry.
4. Run `slam_toolbox`, Nav2 and frontier exploration.
5. Build the Gaussian Splatting pipeline and run the evaluation: ATE/RPE, Chamfer distance, PSNR/SSIM/LPIPS.

## Acknowledgements

The package structure follows the [Articulated Robotics](https://articulatedrobotics.xyz/) "Build a Mobile Robot" series, ported to ROS 2 Jazzy and Gazebo Harmonic.
