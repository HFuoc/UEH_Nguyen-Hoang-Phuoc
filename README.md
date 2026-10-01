# CRC sensor-only driving solution

**Participant:** Nguyễn Hoàng Phước  
**University:** University of Economics Ho Chi Minh City (UEH)  
**School:** School of Technology and Design  
**Institute:** Institute of Intelligent and Interactive Technologies  
**Student ID:** 31231021201  
**Private repository name:** `UEH_Nguyen-Hoang-Phuoc`

This is an AI-assisted Python/ROS 2 Humble baseline for the UEH CRC 2026 simulation round. It implements camera lane following, STOP and traffic-light heuristics, and LiDAR obstacle stopping. Overtaking is disabled. Check `PROGRESS.md` for measured validation status; feature presence is not evidence of successful track completion.

## Environment and installation

Use the official ROS 2 Humble / Gazebo Classic 11 Docker environment. The simulator and solution live in separate ROS packages. Protected world, model and traffic configuration files remain unchanged. The Dockerfile changes package transport to HTTPS for this network.

From the workspace root on Linux:

```bash
bash scripts/run_docker.sh build
bash scripts/run_docker.sh compile
bash scripts/run_docker.sh up-headless
bash scripts/run_docker.sh sh
```

The last command opens the container shell. Use `up` instead of `up-headless` for the Linux desktop GUI. Do not run the official `starter` or teleop at the same time as this driver: only one node may publish motion commands.

For a clean ROS 2 Humble workspace without this Docker workflow, install dependencies, copy `src/crc_solution` into its `src/`, build and source it:

```bash
sudo apt-get install ros-humble-cv-bridge python3-opencv python3-numpy
colcon build --symlink-install --packages-select crc_solution
source install/setup.bash
```

## Single command to start the submitted solution

Run inside the prepared ROS environment **after the simulator is already running**:

```bash
ros2 launch crc_solution run.launch.py
```

The launcher does not start/reset Gazebo, spawn entities, or query ground truth. Match the simulator's ROS domain (the supplied image uses `ROS_DOMAIN_ID=30`).

## Architecture

`camera + camera_info -> lane/sign perception -> behaviour controller -> /cmd_vel`

`scan -> body corridor clearance -> behaviour controller`

`odom -> short-term motion, stationary detection and travelled distance`

The only motion publisher is `/crc_driver`. A wall-clock watchdog stops motion when sensor delivery or the simulation clock stops. STOP holding uses simulation time and measured stationary speed. The controller never reads track coordinates or the official world/model/config files.

Lane boundaries are projected onto a locally flat ground plane using stock camera calibration. A consensus fit rejects isolated crossing marks. A broad bright ramp may supply a visible side boundary when it covers the painted lane. The controller follows the inferred lane centre with a curvature command and reduces speed for uncertainty and turns. Brief missing markings use the last observation adjusted by odometry yaw; sustained loss stops the robot.

Traffic controls use colour/shape and confirmation over multiple images. Light candidates need a dark housing. This lightweight detector is imperfect; inspect annotated evidence rather than assuming all signs are recognized. Only STOP has an implemented sign-specific action. Crossing stripes prompt slower approach; LiDAR governs obstruction stopping. No unvalidated lane change is attempted.

## Runtime parameters and video demonstration

```bash
ros2 param get /crc_driver max_speed
ros2 param set /crc_driver max_speed 0.08
ros2 param set /crc_driver max_speed 0.12
```

Explain that lower speed reduces distance per second and allows gentler motion; it does not improve the camera detector itself. Parameters include `max_speed`, `max_turn`, `steering_gain`, `stop_distance`, `sensor_timeout`, `lane_grace`, and `stop_hold`. Invalid dynamic values are rejected; STOP duration cannot be set below two seconds.

## Evidence and verification

Each run writes CSV telemetry and annotated JPEGs under `/tmp/crc_results/<timestamp>/`. Copy results out before removing the container. Store raw evidence in `results/` locally; it is excluded from Git to avoid large commits.

```bash
python -m unittest discover -s tests -v
python tools/check_rules.py
python analysis/summarize.py results
python analysis/build_report.py
```

Analysis tools run on the host and need NumPy, matplotlib and PyMuPDF (`python -m pip install numpy matplotlib pymupdf`). Runtime does not need matplotlib or PyMuPDF. `analysis/generated/` holds figures and summaries. A compact completed-run data set is retained under `analysis/evidence/`; pass the selected batch directory there to `summarize.py` to reproduce the report. Camera-estimated lateral RMS is **not** the official lane RMS. Wheel odometry distance is **not** completed route distance. No official competition score is estimated.

For live rule checking, run `ros2 node info /crc_driver` and inspect subscriptions; only robot sensors and `/clock` are expected. Official simulator nodes themselves necessarily use Gazebo services; the submitted solution must not.

## Limitations and submission

Camera projection assumes stock camera mounting and approximately level ground; ramps can bias it. Junction branch choice follows visible lane continuity and is not a route planner. STOP/light heuristics can miss small, occluded or oblique objects. A persistently lost red light leaves the robot stopped. Stationary obstacles can prevent further progress because overtaking is disabled.

Before submission, use the private repository name `UEH_Nguyen-Hoang-Phuoc`, add the judges, confirm the actual deadline, complete the personal AI statement and provide the participant's unedited video in `VIDEO.md`. Do not submit a report containing validation claims you have not reproduced.
