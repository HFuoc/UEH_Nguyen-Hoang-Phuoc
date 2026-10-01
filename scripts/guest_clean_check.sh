#!/usr/bin/env bash
set -euo pipefail
cd /home/fish/crc_ws
for previous in $(docker ps -q --filter volume="$PWD/src/crc_solution"); do docker stop "$previous"; done
docker run --rm --net=host -e ROS_DOMAIN_ID=98 -v "$PWD/src/crc_solution":/input:ro \
  crc_sim:humble bash -lc '
    set -e
    source /opt/ros/humble/setup.bash
    mkdir -p /clean/src
    cp -r /input /clean/src/crc_solution
    cd /clean
    colcon build --packages-select crc_solution
    source install/setup.bash
    set +e
    timeout --signal=TERM --kill-after=5s 5s ros2 launch crc_solution run.launch.py > /tmp/launch.log 2>&1
    result=$?
    set -e
    cat /tmp/launch.log
    test "$result" -eq 124
    ! grep -qE "Traceback|process has died" /tmp/launch.log
    echo "Clean container build and one-command launch PASS"
  '
