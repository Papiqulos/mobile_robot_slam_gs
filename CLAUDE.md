# CLAUDE.md

Project context for an autonomous wheeled robot that runs SLAM and builds a Gaussian Splatting digital twin of a room. The goal is a publication. The owner is Giorgos, who already has robotics experience.

## Project goal

Full autonomy, from mapping through exploration to a digital twin, on a low-cost differential-drive robot running ROS 2.

**Working paper idea:** uncertainty-driven autonomous exploration for Gaussian Splatting digital twins on a ground robot, with lidar-aided poses.

- **Robot layer:** 2D lidar SLAM gives an occupancy grid for Nav2. RealSense depth adds 3D obstacles.
- **Twin layer:** RGB images, SLAM poses and depth initialization feed 3DGS.
- **Contribution:** the geometric map tells the robot where the twin still lacks good views. Depth regularizes the Gaussians, keeping walls flat and the scale metric.

**Motivation, as it should be framed:** the point is not nicer splats than a video + COLMAP pipeline. It is autonomous, metric, repeatable twin capture, where the robot decides what to capture. Video + COLMAP + 3DGS is a baseline to compare against.

## Phases

1. **Base:** firmware, `ros2_control` with `diff_drive_controller`, URDF, teleop, odometry, TF tree (`odom → base_link → laser, camera`). Done when odometry closes a 2 m square within a few cm (UMBmark).
2. **SLAM and autonomy:** `slam_toolbox`, `robot_localization` EKF, Nav2, RTAB-Map as a baseline, and frontier exploration with `m-explore-ros2`. Done when the robot maps a room on its own and its ATE is measured.
3. **Gaussian Splatting:** first offline (rosbags → SLAM poses → nerfstudio `splatfacto` / gsplat, compared against COLMAP poses), then online in the SplaTAM / MonoGS style with lidar pose priors.
4. **Contribution:** uncertainty-driven exploration against the frontier baseline, plotting twin quality against exploration time.

**Rules from day one:** log rosbags, calibrate intrinsics and extrinsics, and keep timestamps synchronized.

## Hardware

### Chassis and drive (already owned)
- **Chassis:** ActivMedia AmigoBot, with an aluminum chassis and a polycarbonate body.
  - Size: about 33 × 28 × 13 cm.
  - Mass: 3.6 kg as stock. **Rated payload is only 1 kg.** Replace the lead-acid battery with Li-ion to free up weight.
- **Motors:** 2× Pittman GM8212D142.
  - 12 V brushed DC, G35A spur gearhead at 19.5:1 (efficiency 0.73).
  - 500 CPR encoder: **39,000 counts per wheel revolution** with x4 quadrature, about **124 counts/mm**.
  - The older AmigoBot docs quote 9,550 ticks/rev. That figure belongs to an older encoder generation; ignore it.
  - No public datasheet exists for this custom part. The closest catalog model is GM8212-21, which has a 19.1 V winding, so its electrical constants do not carry over.
  - **To measure:** winding resistance, Kₑ, no-load speed and current at 12 V.
- **Wheels:** 10 cm nominal diameter. Measure the real value, because the docs mix 4 in and 10 cm.
- **Track width:** not documented. Original firmware `RevCount = 23310` suggests about 25 cm. **Measure it.**
- **Original controller:** Hitachi H8 running AmigOS over a 9600–38400 baud serial protocol. It is being replaced.
- **Sonar:** 8 units, available as an optional close-range safety layer. Positions in mm and degrees:
  - Front: (75, 105, 90), (115, 80, 44), (140, 30, 12), (140, −30, −12), (115, −80, −44), (75, −105, −90)
  - Rear: (−140, −82, −144), (−140, 82, −216)

### Electronics (planned)
- **Compute:** Raspberry Pi 5 (8 GB) with an active cooler and an M.2 HAT+ NVMe SSD. No onboard GPU, so **3DGS training runs on a desktop GPU** with ≥12 GB VRAM.
- **MCU:** ESP32 (alternatives: Teensy 4.1, Pico). It handles encoders and wheel PID, connected to the Pi via micro-ROS or serial.
- **Motor driver:** XY-160D.
  - 6.5–27 V supply, 7 A per channel, optocoupler-isolated inputs.
  - L298-style control: ENA (PWM) plus IN1 and IN2 per motor.
  - PWM up to 10 kHz; run it at 8–10 kHz.
  - Control inputs accept 3.3–6.5 V, so 3.3 V logic is at the lower edge. Verify on the bench.
- **Lidar:** Slamtec RPLIDAR C1, a 2D unit. Driver: `sllidar_ros2`.
- **Depth camera:** Intel RealSense D435i, with a built-in IMU. Driver: `realsense-ros`.
- **Battery:** 3S Li-ion with BMS, 10–12.6 V, 5000 mAh or more.

### Power distribution (professor's specification)
One battery feeds three separately fused branches, with a star ground at the distribution point.

- **Common section:** battery with BMS → 10 A main fuse close to the battery positive → DC-rated main switch → blade fuse box, with an INA226 sensor on the trunk.
- **Drive branch:**
  - 5 A slow-blow fuse
  - E-stop (latching, 1NC + 1NO). It cuts **motor power only**; the NO contact goes to an MCU GPIO.
  - XY-160D, with a 470–1000 µF capacitor and an SMBJ15A TVS at its input
  - 100 nF capacitors across each motor's terminals, and twisted motor leads
- **Pi branch:**
  - 3–4 A fuse
  - Buck converter 12 → 5.1 V, ≥5 A, with reverse-polarity protection
  - Controlled shutdown below about 10.5 V
  - Set `usb_max_current_enable=1` in the Pi config
- **MCU and sensors branch:**
  - 2 A fuse
  - Separate buck converter 12 → 5 V, 3 A
  - Feeds the ESP32, the encoders (level-shift to 3.3 V if they output 5 V) and a powered USB hub for the lidar and camera
- **Budget:** about 6–7 A peak in total.

## Software stack
- Ubuntu 24.04 with ROS 2 Jazzy on the Pi 5
- `ros2_control` + `diff_drive_controller`, `robot_localization`, `slam_toolbox`, Nav2, RTAB-Map
- 3DGS: nerfstudio (`splatfacto`), gsplat, the original Kerbl et al. code, SplaTAM, MonoGS, GS-ICP SLAM
- Evaluation tools: `evo` for ATE/RPE, Open3D for Chamfer distance and completeness, COLMAP for reference poses

## Workspace (`ros2_ws/src/mobile_robot`, package name `mobile_robot`, ament_cmake)
- Built from the Articulated Robotics `my_bot` template (written for Foxy + Gazebo Classic) and ported to Jazzy + Gazebo Harmonic. Giorgos is following that tutorial; don't add code beyond what he asks for.

## Firmware notes
- The original AmigOS ran a 10 ms control loop with 5 kHz PWM and **PD control with no integral term** (Kp 40, Kv 80, Ki 0 in AmigOS units). Use that structure as a starting point, not the numbers.
- Reimplement the original safety features:
  - A watchdog that stops the motors if no command arrives for about 2 s.
  - Stall detection: high PWM with no encoder ticks means a stall, so cut motor power.

## Evaluation plan
- **Trajectory:** ATE and RPE.
- **Geometry:** Chamfer distance and completeness against a reference scan.
- **Appearance:** PSNR, SSIM and LPIPS on held-out views.
- **Autonomy:** exploration time, path length, coverage over time.
- **Datasets** for sanity checks: TUM RGB-D, Replica, ScanNet / ScanNet++.
- **Baselines:** frontier exploration with RTAB-Map, SplaTAM without lidar priors, and video + COLMAP + 3DGS.
- **Venues:** IROS/ICRA workshops, RA-L, *Sensors*, *Robotics*.

## Open items
- [ ] Encoder pinout, wire colors and output voltage. An email inquiry to AMETEK Pittman has been drafted; otherwise trace the encoder board.
- [ ] Motor electrical constants and stall current at 12 V.
- [ ] Real wheel diameter and track width.
- [ ] Motor and power connector pinout (8-pin Mini-Fit Jr). Check page 43 of the AmigoBot technical manual; the text extraction was garbled.
- [ ] Bench-test the XY-160D with 3.3 V ESP32 logic.
- [ ] Final drive fuse value, once the stall current is known.

## References
- AmigoBot User's Guide: https://www.generationrobots.com/media/AmigoGuide.pdf
- AmigoBot Technical Manual v1.3: https://powersof2.gatech.edu/resources/DE2Bot/AmigoTech1_3.pdf
- Closest motor datasheet, GM8212-21: https://assets.rs-online.com/v1698741695/Datasheets/af684bc0a16167a861c38d8834ace81e.pdf
- XY-160D product page: https://grobotronics.com/dual-dc-motor-driver-xy-160d-7a-160w.html

## Guidance for Claude
- Never guess pinouts, wire colors or motor constants. Flag them as unknown and suggest how to measure them.
- Keep the 39,000 counts/rev figure and the measured geometry consistent across URDF, `ros2_control` and firmware.
- Prefer standard ROS 2 packages over custom code unless the custom code is the research contribution.
- Respect the 1 kg payload and the power budget when suggesting hardware.
- Giorgos may write in Greek or English; reply in the language he uses.
- Never run git commands
