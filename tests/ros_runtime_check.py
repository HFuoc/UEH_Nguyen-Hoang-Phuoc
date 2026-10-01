"""Integration check with synthetic ROS sensor publishers, not a Gazebo run."""
import math
from pathlib import Path
import sys
import tempfile
import time
import cv2
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
from cv_bridge import CvBridge

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/crc_solution'))
from crc_solution.node import Driver


def main():
    with tempfile.TemporaryDirectory(prefix='crc_ros_test_') as output:
        rclpy.init(args=['--ros-args','-p','use_sim_time:=true','-p',f'log_dir:={output}'])
        driver=Driver(); sensors=Node('synthetic_sensor_test')
        executor=SingleThreadedExecutor(); executor.add_node(driver); executor.add_node(sensors)
        camera=sensors.create_publisher(Image,'/camera/image_raw',qos_profile_sensor_data)
        lidar=sensors.create_publisher(LaserScan,'/scan',qos_profile_sensor_data)
        odom=sensors.create_publisher(Odometry,'/odom',10)
        clock=sensors.create_publisher(Clock,'/clock',10)
        commands=[]
        sensors.create_subscription(Twist,'/cmd_vel',lambda msg:commands.append(msg.linear.x),10)
        img=np.full((480,640,3),20,np.uint8)
        for side in (-.175,.175):
            pts=[(int(320-side*(row-240)/.117),row) for row in range(265,480)]
            cv2.polylines(img,[np.array(pts)],False,(230,230,230),4)
        frame=CvBridge().cv2_to_imgmsg(img,'bgr8')
        scan=LaserScan(); scan.angle_min=-math.pi; scan.angle_increment=math.pi/180
        scan.range_min=.12; scan.range_max=3.5; scan.ranges=[math.inf]*360
        motion=Odometry(); motion.pose.pose.orientation.w=1.
        began=time.monotonic()
        def pump(seconds,images=True,clocks=True):
            end=time.monotonic()+seconds
            while time.monotonic()<end:
                t=time.monotonic()-began
                tick=Clock(); tick.clock.sec=int(t); tick.clock.nanosec=int((t-int(t))*1e9)
                if clocks: clock.publish(tick)
                if images: camera.publish(frame)
                lidar.publish(scan); odom.publish(motion)
                for _ in range(8): executor.spin_once(timeout_sec=.005)
        try:
            pump(2.)
            assert any(v>0 for v in commands), 'No forward command with valid sensors'
            result=driver.set_parameters([Parameter('max_speed',value=.06)])
            assert result[0].successful
            pump(.5)
            assert 0 < commands[-1] <= .06
            result=driver.set_parameters([Parameter('stop_hold',value=1.)])
            assert not result[0].successful
            assert driver.settings.stop_hold>=2.
            pump(1.2,images=False)
            assert commands[-1]==0., 'Camera loss did not stop motion'
            pump(.5)
            assert commands[-1]>0.
            pump(1.2,clocks=False)
            assert commands[-1]==0., 'Paused clock did not stop motion'
            print('ROS integration PASS: motion, parameter update/rejection, camera timeout, clock timeout')
        finally:
            driver.publish(0.,0.); driver.stream.close()
            executor.shutdown(); driver.destroy_node(); sensors.destroy_node(); rclpy.shutdown()


if __name__=='__main__': main()
