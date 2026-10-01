"""Save synchronized-enough onboard observations for offline debugging only."""
import json
import time
from pathlib import Path
import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry

rclpy.init()
node = Node('development_sensor_capture')
data = {}
bridge = CvBridge()
out = Path('/tmp/crc_capture')
out.mkdir(exist_ok=True)
def camera(msg):
    cv2.imwrite(str(out/'camera.jpg'), bridge.imgmsg_to_cv2(msg, 'bgr8'))
    data['image'] = True
def scan(msg):
    data['scan'] = {k: list(msg.ranges) if k == 'ranges' else getattr(msg, k)
                    for k in ('ranges', 'angle_min', 'angle_increment', 'range_min', 'range_max')}
def odom(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    data['odom'] = [p.x, p.y, q.z, q.w]
node.create_subscription(Image, '/camera/image_raw', camera, qos_profile_sensor_data)
node.create_subscription(LaserScan, '/scan', scan, qos_profile_sensor_data)
node.create_subscription(Odometry, '/odom', odom, 10)
end = time.monotonic()+15
while len(data) < 3 and time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=.1)
(out/'sensors.json').write_text(json.dumps(data))
node.destroy_node()
rclpy.shutdown()
assert len(data) == 3, 'Missing sensor stream'
