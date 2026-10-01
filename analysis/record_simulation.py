"""Record actual ROS camera frames and odometry in a lightweight dashboard.

This is an observer, never a motion publisher. Frames use simulation-time
timestamps at a constant 30 fps; missing images are repeated, not invented.
"""
import argparse
import math
import signal
from pathlib import Path
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

class Recorder(Node):
    def __init__(self,path,seconds):
        super().__init__('simulation_recorder')
        self.bridge=CvBridge(); self.start=None; self.next_frame=None
        self.duration=seconds; self.done=False; self.pose=(0.,0.)
        self.distance=0.; self.command=(0.,0.); self.trail=[]
        self.out=cv2.VideoWriter(str(path),cv2.VideoWriter_fourcc(*'mp4v'),30.,(960,540))
        if not self.out.isOpened(): raise RuntimeError('Video encoder could not open')
        self.create_subscription(Image,'/camera/image_raw',self.image,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/odom',self.odom,10)
        self.create_subscription(Twist,'/cmd_vel',self.twist,10)

    def odom(self,msg):
        pose=(msg.pose.pose.position.x,msg.pose.pose.position.y)
        if self.trail: self.distance+=math.dist(pose,self.pose)
        self.pose=pose; self.trail.append(pose)

    def twist(self,msg): self.command=(msg.linear.x,msg.angular.z)

    def image(self,msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if self.start is None: self.start=stamp; self.next_frame=stamp
        elapsed=stamp-self.start
        if elapsed > self.duration: self.done=True; return
        frame=self.bridge.imgmsg_to_cv2(msg,'bgr8')
        canvas=np.full((540,960,3),(24,27,32),np.uint8)
        canvas[45:525,15:655]=cv2.resize(frame,(640,480))
        def text(value,x,y,size=.55,color=(225,225,225)):
            cv2.putText(canvas,value,(x,y),cv2.FONT_HERSHEY_SIMPLEX,size,color,1,cv2.LINE_AA)
        text('UEH CRC 2026 | Ubuntu / ROS 2 Humble | Simulation recording',15,27,.65)
        for y,label in [(72,'Nguyen Hoang Phuoc'),(98,'Student ID: 31231021201'),
                        (139,f'Simulation: {elapsed:6.1f} s'),(166,f'Odometry travel: {self.distance:.2f} m'),
                        (198,f'Command v: {self.command[0]:.3f} m/s'),(225,f'Command w: {self.command[1]:.3f} rad/s'),
                        (261,'Wheel-odometry trace')]: text(label,680,y)
        if self.trail:
            pts=np.asarray(self.trail); span=np.maximum(np.ptp(pts,axis=0),1.)
            scale=min(235/span[0],190/span[1]); center=(pts.max(axis=0)+pts.min(axis=0))/2
            pixels=((pts-center)*[scale,-scale]+[812,380]).astype(np.int32)
            cv2.polylines(canvas,[pixels],False,(120,185,230),2,cv2.LINE_AA)
            cv2.circle(canvas,tuple(pixels[-1]),4,(60,170,250),-1)
        text('Recorded camera + onboard odometry',680,508,.40)
        text('No route-completion claim',680,528,.40)
        while self.next_frame <= stamp:
            self.out.write(canvas); self.next_frame+=1/30.

def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--output',required=True)
    parser.add_argument('--seconds',type=float,default=300.)
    args=parser.parse_args(); Path(args.output).parent.mkdir(parents=True,exist_ok=True)
    cv2.setNumThreads(1); rclpy.init(); node=Recorder(args.output,args.seconds)
    signal.signal(signal.SIGTERM,lambda *_: setattr(node,'done',True))
    try:
        while rclpy.ok() and not node.done: rclpy.spin_once(node,timeout_sec=.2)
    finally:
        node.out.release(); node.destroy_node(); rclpy.shutdown()

if __name__=='__main__': main()
