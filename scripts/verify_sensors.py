"""Read only sensor smoke test, executed inside the ROS container."""
import json
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan, CameraInfo
from nav_msgs.msg import Odometry

rclpy.init()
node=Node('crc_sensor_verification')
counts={name:0 for name in ('image','scan','odom','info')}
def callback(name):
    def count(message): counts[name]+=1
    return count
for name,topic,kind in [('image','/camera/image_raw',Image),('scan','/scan',LaserScan),
                        ('odom','/odom',Odometry),('info','/camera/camera_info',CameraInfo)]:
    node.create_subscription(kind,topic,callback(name),qos_profile_sensor_data)
deadline=time.monotonic()+90
while time.monotonic()<deadline and min(counts.values())<3:
    rclpy.spin_once(node,timeout_sec=.2)
print(json.dumps(counts),flush=True)
node.destroy_node(); rclpy.shutdown()
raise SystemExit(0 if min(counts.values())>=3 else 1)
