"""ROS sensor adapter, fail-safe command output and evidence logging."""
import csv
from datetime import datetime, timezone
import math
from pathlib import Path
import signal
import time

import cv2
import numpy as np
from cv_bridge import CvBridge
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, LaserScan

from .control import Controller, Settings, PARAMETER_LIMITS, MIN_LANE_CONFIDENCE
from .perception import Perception, Lane, Signs
from .navigation import scan_points, choose_curvature, steering_curvature, enclosed_corridor
from .freshness import SensorFreshness
from .pedestrians import CrossingGuard, detect_people
from .sign_classifier import SignClassifier
from .sign_actions import SignActions


class Driver(Node):
    def __init__(self):
        super().__init__('crc_driver')
        defaults = Settings()
        for key, value in vars(defaults).items():
            self.declare_parameter(key, value)
        self.declare_parameter('log_dir', '/tmp/crc_results')
        self.settings = Settings(**{k:self.get_parameter(k).value for k in vars(defaults)})
        self.settings.validate()
        self.add_on_set_parameters_callback(self.parameters_changed)
        self.controller = Controller(self.settings)
        cv2.setNumThreads(1)
        self.perception, self.bridge = Perception(), CvBridge()
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Image, '/camera/image_raw', self.on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_info, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.freshness = SensorFreshness()
        self.scan_points = None
        self.scan_message = None
        self.curvature = 0.
        self.crossing = CrossingGuard()
        self.people = []
        self.sign_classifier = SignClassifier()
        self.sign_actions = SignActions()
        self.classified_signs = []
        self.image = None
        self.image_sequence = self.processed_sequence = 0
        self.lane, self.signs = Lane(), Signs()
        self.front = math.nan
        self.yaw = self.speed = self.distance = 0.
        self.position = None
        self.last_clock = None
        self.clock_changed_wall = time.monotonic()
        self.last_save = self.last_log = -math.inf
        self.last_log_wall = -math.inf
        self.last_process_wall = -math.inf
        self.previous_state = ''
        run = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
        self.output = Path(self.get_parameter('log_dir').value).expanduser()/run
        self.output.mkdir(parents=True, exist_ok=False)
        self.stream = (self.output/'telemetry.csv').open('w', newline='')
        self.writer = csv.writer(self.stream)
        self.writer.writerow(['sim_time','wall_time','state','distance_odom_m','x_odom','y_odom',
                              'speed_measured','command_v','command_w','lane_lateral_est_m',
                              'lane_heading_est_rad','lane_confidence','front_m','light','stop',
                              'sign_label','people_count','path_curvature',
                              'image_wall_age','scan_wall_age','odom_wall_age','clock_wall_age',
                              'image_stamp_age','scan_stamp_age','odom_stamp_age','processing_ms',
                              'light_distance_m'])
        # A wall timer keeps publishing zero if /clock stops or sensor flow dies.
        self.timer = self.create_timer(.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(f'Evidence: {self.output}')

    def parameters_changed(self, parameters):
        limits = PARAMETER_LIMITS
        updates = {}
        for parameter in parameters:
            if parameter.name in limits:
                lo, hi = limits[parameter.name]
                value = parameter.value
                if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not lo <= value <= hi:
                    return SetParametersResult(successful=False, reason=f'{parameter.name}: range {lo}..{hi}')
                updates[parameter.name] = float(value)
        for key, value in updates.items():
            setattr(self.settings, key, value)
        return SetParametersResult(successful=True)

    def on_info(self, msg):
        self.perception.calibrate(msg.k)

    def on_image(self, msg):
        try:
            if not self.accept_stamp('image', msg):
                return
            self.image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.image_sequence += 1
        except Exception as exc:
            self.freshness.invalidate('image')
            self.publish(0., 0.)
            self.get_logger().error(f'Image conversion failed: {exc}', throttle_duration_sec=5.)

    def on_scan(self, msg):
        if not self.accept_stamp('scan', msg):
            return
        self.scan_message = msg
        self.scan_points = scan_points(msg.ranges, msg.angle_min, msg.angle_increment,
                                       msg.range_min, msg.range_max)

    def accept_stamp(self, name, msg):
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        now = self.get_clock().now().nanoseconds/1e9
        return self.freshness.accept(name, stamp, time.monotonic(), now=now)

    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        if not all(math.isfinite(v) for v in (p.x,p.y,q.x,q.y,q.z,q.w,msg.twist.twist.linear.x)):
            self.freshness.invalidate('odom')
            return
        if not self.accept_stamp('odom', msg):
            return
        if self.position is not None:
            delta = math.hypot(p.x-self.position[0], p.y-self.position[1])
            if delta < .25:
                self.distance += delta
        self.position = (p.x, p.y)
        self.yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.speed = msg.twist.twist.linear.x

    def publish(self, speed, turn):
        message = Twist()
        message.linear.x, message.angular.z = float(speed), float(turn)
        self.pub.publish(message)

    def tick(self):
        try:
            self.control_tick()
        except Exception as exc:
            self.publish(0., 0.)
            self.get_logger().error(f'Fail-safe stop: {exc}', throttle_duration_sec=5.)

    def control_tick(self):
        now, wall = self.get_clock().now().nanoseconds/1e9, time.monotonic()
        if self.last_clock is None or now != self.last_clock:
            if self.last_clock is not None and now < self.last_clock:
                self.controller = Controller(self.settings)
                self.distance, self.position = 0., None
                self.freshness.clear()
                self.perception = Perception()
                self.lane, self.signs = Lane(), Signs()
                self.scan_points = None
                self.crossing = CrossingGuard()
                self.people = []
                self.sign_actions = SignActions()
                self.classified_signs = []
                self.last_save = self.last_log = -math.inf
            self.last_clock, self.clock_changed_wall = now, wall
        fresh = (self.freshness.fresh(now, wall, self.settings.sensor_timeout)
                 and wall-self.clock_changed_wall < self.settings.sensor_timeout)
        if self.image_sequence != self.processed_sequence and fresh and wall-self.last_process_wall >= .1:
            self.lane, _ = self.perception.lane(self.image)
            self.signs = self.perception.signs(self.image)
            self.people = detect_people(self.image, self.perception)
            self.sign_classifier.fx = self.perception.fx
            self.classified_signs = self.sign_classifier.detect(self.image)
            self.sign_actions.update(self.classified_signs,now,self.distance)
            for detected in self.classified_signs:
                if detected.label=='stop':
                    self.signs.stop=True
                    self.signs.stop_distance=min(self.signs.stop_distance,detected.distance)
                # A confidently recognized printed plate is not a lamp,
                # including when it is seen against a dark background.
                x,y,w,h=detected.bbox
                for bx,by,bw,bh,label in self.signs.boxes:
                    overlap=max(0,min(x+w,bx+bw)-max(x,bx))*max(0,min(y+h,by+bh)-max(y,by))
                    if label in ('red','yellow','green') and overlap > .5*bw*bh:
                        self.signs.light='unknown'
            self.controller.observe(self.signs, now, self.distance)
            self.processed_sequence = self.image_sequence
            self.last_process_wall = wall
        lookahead = .30 if enclosed_corridor(self.scan_points) else .45
        desired = steering_curvature(self.lane, self.settings.steering_gain, lookahead)
        if self.lane.confidence < MIN_LANE_CONFIDENCE:
            desired = self.controller.last_path_curvature
        self.curvature, self.front = choose_curvature(self.scan_points,
            desired, self.curvature)
        speed, turn = self.controller.step(now, self.distance, self.yaw, self.speed,
                                            self.lane, self.front, fresh, self.curvature)
        if self.signs.crosswalk and speed > .07:
            turn *= .07/speed
            speed = .07
        cap=self.sign_actions.speed_cap(self.distance)
        if speed > cap:
            turn *= cap/speed
            speed = cap
        if fresh and self.crossing.update(self.people, self.scan_points, self.lane, now):
            speed, turn = 0., 0.
            self.controller.state = 'CROSSING_WAIT'
        if not self.freshness.fresh(now, time.monotonic(), self.settings.sensor_timeout):
            speed, turn = 0., 0.
            self.controller.state = 'WAIT_SENSORS'
        self.publish(speed, turn)
        state = self.controller.state
        changed = state != self.previous_state
        if changed:
            self.get_logger().info(f'{state}: odom distance={self.distance:.2f} m')
            self.previous_state = state
        if now-self.last_log >= .10 or wall-self.last_log_wall >= .5 or changed:
            x, y = self.position or (0., 0.)
            self.writer.writerow([now,wall,state,self.distance,x,y,self.speed,speed,turn,
                                  self.lane.lateral,self.lane.heading,self.lane.confidence,
                                  self.front,self.controller.light_color,self.signs.stop,
                                  self.sign_actions.label,len(self.people),self.curvature,
                                  *[wall-self.freshness.received.get(k, -math.inf) for k in ('image','scan','odom')],
                                  wall-self.clock_changed_wall,
                                  *[now-self.freshness.stamps.get(k, -math.inf) for k in ('image','scan','odom')],
                                  (time.monotonic()-wall)*1000., self.controller.light_distance])
            self.stream.flush()
            self.last_log = now
            self.last_log_wall = wall
        if self.image is not None and (now-self.last_save >= 5. or changed):
            if self.scan_points is not None:
                np.savez_compressed(self.output/f'{now:010.2f}_scan.npz', points=self.scan_points,
                                    yaw=self.yaw, position=self.position)
            cv2.imwrite(str(self.output/f'{now:010.2f}_camera.jpg'),self.image)
            if changed:
                # Preserve exact detector pixels at state transitions; JPEG
                # alters small lamp boundaries enough to hide regressions.
                cv2.imwrite(str(self.output/f'{now:010.2f}_camera.png'),self.image)
            frame = self.image.copy()
            for x,y in self.lane.points:
                cv2.circle(frame,(x,y),4,(0,255,255),-1)
            for x,y,w,h,label in self.signs.boxes:
                cv2.rectangle(frame,(x,y),(x+w,y+h),(255,180,0),1)
                cv2.putText(frame,label,(x,max(12,y-3)),cv2.FONT_HERSHEY_SIMPLEX,.4,(255,255,255),1)
            for person in self.people:
                x,y,w,h=person.box
                cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,255),2)
            for sign in self.classified_signs:
                x,y,w,h=sign.bbox
                cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),1)
                cv2.putText(frame,sign.label,(x,max(12,y-3)),cv2.FONT_HERSHEY_SIMPLEX,.4,(0,255,0),1)
            cv2.putText(frame,f'{state} v={speed:.2f} c={self.lane.confidence:.2f}',
                        (8,22),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,255,255),1)
            cv2.imwrite(str(self.output/f'{now:010.2f}_{state}.jpg'),frame)
            self.last_save = now


def main(args=None):
    rclpy.init(args=args)
    stopping = [False]
    signal.signal(signal.SIGTERM, lambda *_: stopping.__setitem__(0, True))
    node = Driver()
    try:
        while rclpy.ok() and not stopping[0]:
            rclpy.spin_once(node, timeout_sec=.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.publish(0., 0.)
        node.stream.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
