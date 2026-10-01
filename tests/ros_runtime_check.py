"""Integration check with synthetic ROS sensor publishers, not a Gazebo run."""
import math
import os
from pathlib import Path
import subprocess
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
        child=None
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
        def pump(seconds,images=True,clocks=True,frozen_stamps=False,stamp_offset=0.,scans=True):
            end=time.monotonic()+seconds
            while time.monotonic()<end:
                t=time.monotonic()-began
                tick=Clock(); tick.clock.sec=int(t); tick.clock.nanosec=int((t-int(t))*1e9)
                if not frozen_stamps:
                    stamped=Clock()
                    source_time=t+stamp_offset
                    stamped.clock.sec=int(source_time)
                    stamped.clock.nanosec=int((source_time-int(source_time))*1e9)
                    frame.header.stamp = stamped.clock
                    scan.header.stamp = stamped.clock
                    motion.header.stamp = stamped.clock
                if clocks: clock.publish(tick)
                if images: camera.publish(frame)
                if scans: lidar.publish(scan)
                odom.publish(motion)
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
            pump(1.2,scans=False)
            assert commands[-1]==0., 'LiDAR loss did not stop motion'
            pump(.5)
            assert commands[-1]>0., 'Valid LiDAR did not recover'
            scan.ranges=[math.nan]*360
            pump(.4)
            assert commands[-1]==0., 'Invalid LiDAR ranges did not stop motion'
            scan.ranges=[math.inf]*360
            pump(.5)
            assert commands[-1]>0., 'Clear LiDAR did not recover after invalid ranges'
            pump(1.2,frozen_stamps=True)
            assert commands[-1]==0., 'Repeated stale sensor stamps did not stop motion'
            pump(.5)
            assert commands[-1]>0.
            pump(1.2,stamp_offset=100000.)
            assert commands[-1]==0., 'Future-stamped sensors did not time out'
            assert all(stamp < 100000. for stamp in driver.freshness.stamps.values()), 'Future stamp was stored'
            pump(.5)
            assert commands[-1]>0., 'Valid sensors did not recover after future timestamp outliers'
            pump(1.2,clocks=False)
            assert commands[-1]==0., 'Paused clock did not stop motion'
            # Exercise the actual executable's signal handler, not a direct
            # call to publish(0): a normal TERM must stop the robot and exit.
            executor.remove_node(driver)
            driver.stream.close(); driver.destroy_node(); driver=None
            environment=dict(os.environ)
            source=str(Path(__file__).resolve().parents[1]/'src/crc_solution')
            environment['PYTHONPATH']=source+os.pathsep+environment.get('PYTHONPATH','')
            child=subprocess.Popen([sys.executable,'-c','from crc_solution.node import main; main()',
                '--ros-args','-p','use_sim_time:=true','-p',f'log_dir:={output}'],
                env=environment,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
            commands.clear()
            pump(3.)
            assert commands and commands[-1]>0., 'Executable did not drive with valid sensors'
            before=len(commands)
            child.terminate()
            pump(.8)
            trace=child.communicate(timeout=5.)[0]
            assert child.returncode==0, trace
            assert len(commands)>before and commands[-1]==0., 'TERM did not publish a final zero command'
            print('ROS integration PASS: motion, parameters, camera/LiDAR loss, invalid ranges, stale stamps, future-stamp recovery, clock timeout, executable TERM stop')
        finally:
            if child is not None and child.poll() is None:
                child.kill(); child.communicate(timeout=5.)
            if driver is not None:
                driver.publish(0.,0.); driver.stream.close(); driver.destroy_node()
            executor.shutdown(); sensors.destroy_node(); rclpy.shutdown()


if __name__=='__main__': main()
