#!/usr/bin/env bash
set -euo pipefail
docker run --rm osrf/ros:humble-desktop-full bash -lc 'for url in https://repo.ros2.org/ubuntu/main/dists/jammy/InRelease https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu/dists/jammy/InRelease https://ftp.osuosl.org/pub/ros2/ubuntu/dists/jammy/InRelease; do echo "$url"; curl -IL --max-time 15 "$url" 2>/dev/null | head -5; done'
