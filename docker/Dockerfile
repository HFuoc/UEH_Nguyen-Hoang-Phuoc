# UEH CRC 2026 simulation environment - ROS 2 Humble + Gazebo Classic 11
FROM osrf/ros:humble-desktop-full

SHELL ["/bin/bash", "-c"]

RUN find -L /etc/apt -maxdepth 2 -type f \( -name '*.list' -o -name '*.sources' \) -exec sed -i 's|http://archive.ubuntu.com|https://archive.ubuntu.com|g; s|http://security.ubuntu.com|https://security.ubuntu.com|g; s|http://packages.ros.org/ros2/ubuntu|https://repo.ros2.org/ubuntu/main|g' {} + \
    && apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-gazebo-ros-pkgs \
      ros-humble-gazebo-plugins \
      ros-humble-robot-state-publisher \
      ros-humble-joint-state-publisher \
      ros-humble-xacro \
      ros-humble-cv-bridge \
      ros-humble-image-transport \
      ros-humble-teleop-twist-keyboard \
      ros-humble-turtlebot3-description \
      ros-humble-rviz2 \
      ros-humble-rqt-image-view \
      python3-colcon-common-extensions \
      python3-opencv \
      xvfb \
    && rm -rf /var/lib/apt/lists/*

ENV PYTHONDONTWRITEBYTECODE=1
ENV TURTLEBOT3_MODEL=waffle
ENV ROS_DOMAIN_ID=30

# Source is bind-mounted at /ws/src at run time; build artefacts go to a
# separate volume so the source tree stays clean.
# /tmp/.X11-unix is missing from the base image. Xvfb cannot create its
# socket without it, and xvfb-run then waits forever for a server that
# never comes up. The GUI path bind-mounts over this directory anyway.
RUN mkdir -p /ws/src /ws_build /tmp/.X11-unix && chmod 1777 /tmp/.X11-unix
WORKDIR /ws

RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo '[ -f /ws_build/install/setup.bash ] && source /ws_build/install/setup.bash' >> /root/.bashrc && \
    echo 'alias build="colcon build --symlink-install --build-base /ws_build/build --install-base /ws_build/install"' >> /root/.bashrc

CMD ["/bin/bash"]
