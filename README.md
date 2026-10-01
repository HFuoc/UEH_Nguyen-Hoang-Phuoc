# UEH CRC 2026 Autonomous Driving

**Participant:** Nguyen Hoang Phuoc  
**University:** University of Economics Ho Chi Minh City  
**College:** College of Technology and Design  
**Institute:** Institute of Intelligent & Interactive Technologies  
**Student ID:** 31231021201

**Private repository:** [UEH_Nguyen-Hoang-Phuoc](https://github.com/HFuoc/UEH_Nguyen-Hoang-Phuoc)

The `crc_solution` ROS 2 package uses camera images, camera calibration, LiDAR and wheel odometry to follow the lane and handle traffic controls. One node publishes `/cmd_vel`.

## Installation

Use the organiser's Ubuntu 22.04 / ROS 2 Humble / Gazebo Classic 11 environment. Copy `src/crc_solution` into the simulator workspace's `src/` directory. Install the additional dependencies, then build and source the workspace:

```bash
sudo apt-get update
sudo apt-get install ros-humble-cv-bridge python3-opencv python3-numpy
colcon build --symlink-install --packages-select crc_solution
source install/setup.bash
```

Run these commands inside the official container if using the supplied Docker environment. The solution and simulator must share the ROS domain; the supplied image uses `ROS_DOMAIN_ID=30`.

## Start the solution

After the simulator is running, use this command in the prepared ROS environment:

```bash
ros2 launch crc_solution run.launch.py
```

Run one driving node at a time. Stop any teleoperation or starter node before launching the solution.

## Behaviour and parameters

The camera estimates lane position, heading and confidence, and detects traffic controls and the supplied pedestrian. LiDAR checks the swept robot body and wheel footprint. The controller reduces speed in turns and caution zones, waits at traffic controls and crossings, and stops when observations are stale or lane guidance is lost.

Parameters include `max_speed`, `max_turn`, `steering_gain`, `stop_distance`, `sensor_timeout`, `lane_grace` and `stop_hold`. Updates are validated at runtime. The default maximum forward speed is 0.18 m/s.

```bash
ros2 param set /crc_driver max_speed 0.08
ros2 param set /crc_driver max_speed 0.18
```

## Validation and limitations

The selected START evaluation recorded 21.9992 m of wheel-odometry travel over a 300-second simulation interval. The recording ended while the robot was still moving. Another run ended with lane loss after 19.2305 m. This is accumulated travel, not an official completion distance or score. Overtaking is not implemented.

The selected source passes 71 unit/regression tests, ROS sensor-fault checks and a clean-container build and launch. Run tests from the repository root:

```bash
python3 -m unittest discover -s tests -q
```

`REPORT.pdf` explains the algorithms, evidence and limitations. `VIDEO.md` links to the simulation recording. Recorded telemetry and figure-generation scripts are in `analysis/`. Regenerate the figures with:

```bash
python3 -m pip install matplotlib
python3 analysis/submission_figures.py --case video_v15
```
