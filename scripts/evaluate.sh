#!/usr/bin/env bash
# Run against an already-running simulator. Arguments are simulation seconds.
set -euo pipefail
duration="${1:-300}"
set +u
source /opt/ros/humble/setup.bash
source /ws_build/install/setup.bash
set -u
export CRC_RUN_DURATION="$duration"
ros2 launch crc_solution run.launch.py > /tmp/crc-driver.log 2>&1 &
driver_pid=$!
(sleep 4; timeout 12 ros2 node info /crc_driver > /tmp/crc-node-info.txt 2>&1) &
trap 'kill -TERM "$driver_pid" 2>/dev/null || true; wait "$driver_pid" 2>/dev/null || true' EXIT
python3 - <<'PY'
import os,time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
rclpy.init(); n=Node('evaluation_timer')
start=[None]; elapsed=[0.]
last_progress=[0.]; last_position=[None]
def clock(m):
    t=m.clock.sec+m.clock.nanosec/1e9
    if start[0] is None: start[0]=t
    elapsed[0]=t-start[0]
n.create_subscription(Clock,'/clock',clock,qos_profile_sensor_data)
def odom(m):
    p=m.pose.pose.position
    if last_position[0] is None or ((p.x-last_position[0][0])**2+(p.y-last_position[0][1])**2)**.5>.02:
        last_position[0]=(p.x,p.y)
        last_progress[0]=elapsed[0]
n.create_subscription(Odometry,'/odom',odom,qos_profile_sensor_data)
deadline=time.monotonic()+2400
while elapsed[0]<float(os.environ['CRC_RUN_DURATION']) and time.monotonic()<deadline:
    rclpy.spin_once(n,timeout_sec=.2)
    if elapsed[0]-last_progress[0]>45:
        print('Stopped evaluation after 45 simulation seconds without progress.',flush=True)
        break
print('Recorded simulation seconds:',elapsed[0],flush=True)
n.destroy_node(); rclpy.shutdown()
if time.monotonic()>=deadline: raise SystemExit('Wall-clock limit reached; inspect simulator speed.')
PY
