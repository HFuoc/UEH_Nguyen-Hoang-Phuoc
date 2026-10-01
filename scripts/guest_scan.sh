#!/usr/bin/env bash
set -euo pipefail
docker exec -i crc bash -c 'source /opt/ros/humble/setup.bash; python3 -' <<'PY'
import rclpy,math,json,time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
rclpy.init();n=Node('scan_diagnostic');done=[False]
def scan(m):
    candidates=[]
    for i,r in enumerate(m.ranges):
        if not math.isfinite(r) or r<m.range_min or r>m.range_max:continue
        a=m.angle_min+i*m.angle_increment;x=r*math.cos(a)-.064;y=r*math.sin(a)
        if x>0 and abs(y)<.19:candidates.append((round(x,4),round(y,4),round(r,4),round(math.degrees(a),1)))
    print(json.dumps(sorted(candidates)[:16]),flush=True);done[0]=True
n.create_subscription(LaserScan,'/scan',scan,qos_profile_sensor_data)
deadline=time.monotonic()+10
while not done[0] and time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.2)
n.destroy_node();rclpy.shutdown()
PY
