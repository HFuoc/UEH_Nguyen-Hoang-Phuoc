# Runtime checkpoint (2026-10-02)

Priority: improve START-to-FINISH driving and keep a runnable ZIP available. Report work is postponed.

## Available package

`UEH_Nguyen-Hoang-Phuoc_runtime.zip` is built by `python tools/package_submission.py`; its companion `.sha256` and internal manifest verify every file. The current runtime source matches the fingerprint of `full_v13` exactly. REPORT.pdf and the older `_submission.zip` describe the historical 9b22639 baseline.

## Verified checkpoint

- `full_v13`: **15.514 m wheel-odometry path length**, ended LANE_LOST after the tunnel/hairpin. Evaluation ended at 256.8 simulation seconds after no progress. **Not START-to-FINISH, not official route distance or score.** Evidence and source hashes: `analysis/evidence/full_v13/`.
- `full_v12`: 15.488 m over the 300-second evaluation; similar later lane loss. Historical baseline stopped at ~8.85 m at the tunnel bend.
- 69 unit/regression tests pass on Windows and Linux. ROS integration checks sensor loss, invalid LiDAR, stale/future stamps, paused clock, live parameters and the executable's final stop on TERM. Clean-container build and one-command launch pass.
- Protected assets: 19 SHA-256 checks match. Live graph audit passes. No ground-truth/map queries in the runtime solution.
- Fixes include clipped STOP/lamp confusion, stale green entry, STOP rearming, ramp self returns, curved body/wheel collision checking, bounded paint-gap prediction and watchdog recovery. Added packaged eight-sign appearance matching and pedestrian crossing holds.
- Conservative max_speed default is now 0.18 m/s; turning, low confidence and caution zones reduce it. This is the value exercised by full_v13.

## Current limitation and next work

The camera can confuse the lane boundaries after the hairpin, particularly when entering the BUS bend off-centre. A targeted trial from a better centred pose (`bus_right_v4`) continued 4.339 m over 120 seconds, but this did not transfer reliably to a complete START run. A later heading-fit experiment failed and was removed from the checkpoint.

A camera-only right-boundary tracing prototype is being investigated in `.local/trace_lane.py`. Do not package it unless runtime testing shows improvement. Overtaking remains absent. Full-map completion, repeated-run robustness of the new revision, official RMS and collision counts remain unverified.

No development trial is currently required to keep running. VM workspace: `/home/fish/crc_ws`. Use unique case names with `scripts/guest_develop.sh`; never sync runtime source while its evaluation is active. Check termination reason in `evaluation.json`, not just exit_code.

# Baseline status

Private repository: https://github.com/HFuoc/UEH_Nguyen-Hoang-Phuoc

Participant: Nguyen Hoang Phuoc, UEH, School of Technology and Design,
Institute of Intelligent and Interactive Technologies, student ID 31231021201.

## Implemented and verified

- Ubuntu VMware guest configured for 8 GB RAM / 8 vCPU; original VMX backed up beside it.
- Docker, ROS 2 Humble and Gazebo Classic 11 installed. Workspace: `/home/fish/crc_ws`.
- Separate `crc_solution` package: lane following, STOP hold, light wait, ramp-edge fallback, LiDAR stop, bounded lane prediction and sensor/clock watchdogs.
- Dynamic `max_speed` works; invalid values are rejected.
- 19 unit/regression tests passed on Windows and in the Linux image.
- ROS integration passed, including missing-camera and paused-clock stopping.
- Fresh container build and single-command launch passed. A fresh GitHub clone has identical runtime source and passes the protected-asset check. No separate physical-machine validation is claimed.
- 19 protected official files retain their SHA-256 values; all five recorded live ROS graphs pass the interface audit.

## Final evaluation

Batch: `20261001T052745Z`. Source fingerprint and reproducible evidence are in
`analysis/evidence/20261001T052745Z/`. All runs use scale 1.0 and the official
light controller's default seed 0. Physical props, signs, lights and onboard
sensors are enabled; the unused overhead camera is disabled. Maximum duration
is 300 simulation seconds, with early termination after 45 seconds without
meaningful movement.

| Case | Wheel-odometry distance | End condition |
| --- | ---: | --- |
| Default 1 | 8.850 m | Obstacle wait at tunnel bend |
| Default 2 | 8.846 m | Obstacle wait at tunnel bend |
| Default 3 | 8.853 m | Obstacle wait at tunnel bend |
| Shifted start A | 8.807 m | Obstacle wait at tunnel bend |
| Shifted start B | 8.887 m | Obstacle wait at tunnel bend |

Median of the default runs: **8.850 m of wheel-odometry path length**. This is
not completed route distance or an official competition score. All five logs
contain STOP_HOLD and LIGHT_WAIT, but no independently verified traffic-score
or collision count is claimed. Multi-seed testing remains unverified.

## Known limits and handoff

- The straight LiDAR corridor stops the robot at a sharp tunnel bend. The route is not completed.
- Overtaking and full recognition/actions for all eight sign types are not implemented.
- Camera ground projection is approximate on inclines. Changed start A briefly loses the lane before reacquiring it.
- `REPORT.pdf` contains 11 pages with measured results and limitations. It remains a draft pending personal review and the AI declaration.
- `HUONG_DAN.md` explains running the demo in Vietnamese; `VIDEO.md` provides the required six-minute one-take outline.
- Still required from the participant: record/upload the video, review the report, confirm the submission deadline, provide judge accounts, and submit using the organizer's form. No submission or judge invitation has been sent.
