# Progress

- Official simulator extracted unchanged; protected assets recorded in official_checksums.json.
- Existing Ubuntu 20.04 VM started with 8 GB RAM / 8 vCPU. Original VM config backed up next to the VMX.
- Docker installed from Ubuntu repositories. User freed host space; image built successfully.
- ROS/Gazebo image built using signed Ubuntu/ROS repositories over HTTPS. Protected assets unchanged.
- Both ROS packages compile. Real camera, camera_info, LiDAR and odometry smoke checks pass.
- Synthetic ROS integration passes: forward motion, dynamic parameter update/rejection, camera watchdog, clock watchdog.
- Fresh container build and single-command launch pass (not a separate physical machine).
- First development trial stopped after 0.32 m. Near-lane weighting fixed the observed taper issue.
- Second development trial drove about 2.3 wheel-odometry metres in 60 simulation seconds and exercised STOP. This is not official route completion.
- Real imagery exposed desaturated lamp colours; thresholds adjusted and a regression fixture retained.
- Five final evaluation cases (three defaults, two changed starts) are being recorded, maximum 300 simulation seconds, early stop after 45 seconds without progress.
- Target: robust lane control, STOP/light handling, obstacle stop, reproducible evidence.
- Overtaking is not enabled until validated. No competition score is claimed.
- Participant: Nguyen Hoang Phuoc, UEH, School of Technology and Design, Institute of Intelligent and Interactive Technologies, student ID 31231021201.
- Pending confirmed submission deadline, judge accounts and participant-recorded video.
