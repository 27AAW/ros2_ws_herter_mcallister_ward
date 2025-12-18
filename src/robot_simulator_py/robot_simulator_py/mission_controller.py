#!/usr/bin/env python3

import math
from typing import Dict, Tuple

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge
import numpy as np

def normalize_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))

class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')

        # FSM states
        self.state = 'STANDBY'  # STANDBY, SEARCH_ARUCO, MOVE_TO_MARKER, RETURN_TO_ORIGIN

        # Data
        self.current_goal_id = None
        self.marker_poses: Dict[int, Tuple[float, float, float]] = {}
        self.current_pose = (0.0, 0.0, 0.0)  # x, y, yaw
        self.search_start_time = None
        self.origin_pose = None  # will be set from first odom

        # NEW: Square search variables
        self.square_phase = 0  # 0=forward, 1=turn90, 2=forward, 3=turn90+spin
        self.phase_start_time = None
        self.phase_distance = 0.0
        self.square_side = 1.0  # 1m sides

        # OpenCV / camera
        self.bridge = CvBridge()
        # Use OpenCV's built-in ArUco dictionaries
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()

        # Subscribers
        self.missions_sub = self.create_subscription(
            String, '/missions', self.missions_callback, 10
        )
        self.eval_sub = self.create_subscription(
            String, '/evaluator_message', self.evaluator_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.image_sub = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10
        )  # change topic if needed

        # Publishers
        self.status_pub = self.create_publisher(String, '/robot_status', 10)
        self.report_pub = self.create_publisher(String, '/robot_report', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # FSM timer
        self.timer = self.create_timer(0.1, self.state_machine_step)

        # Initialization: publish "ready"
        self.publish_status('ready')
        self.get_logger().info('MissionController started, state=STANDBY')

    # ------------- Callbacks -------------

    def missions_callback(self, msg: String):
        text = msg.data.strip()
        self.get_logger().info(f'/missions: "{text}"')

        if text.startswith('search_aruco'):
            self.state = 'SEARCH_ARUCO'
            self.search_start_time = self.get_clock().now()
            self.publish_status('searching')

        elif text.startswith('move to'):
            parts = text.split()
            if len(parts) == 3 and parts[2].isdigit():
                self.current_goal_id = int(parts[2])
                self.state = 'MOVE_TO_MARKER'
                self.publish_status(f'moving_to_{self.current_goal_id}')
            else:
                self.get_logger().warn('Bad "move to" mission format')

        elif text == 'return_to_origin':
            self.state = 'RETURN_TO_ORIGIN'
            self.publish_status('returning')

    def evaluator_callback(self, msg: String):
        text = msg.data.strip().lower()
        self.get_logger().info(f'/evaluator_message: "{text}"')

        if 'success' in text:
            if self.state == 'MOVE_TO_MARKER':
                self.state = 'RETURN_TO_ORIGIN'
                self.publish_status('returning')
        elif 'fail' in text or 'adjust' in text:
            self.publish_status('adjusting_pose')

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.current_pose = (x, y, yaw)

        # Store origin only once
        if self.origin_pose is None:
            self.origin_pose = (x, y, yaw)

    def image_callback(self, msg: Image):
        # Only care about images when searching for ArUco markers
        if self.state != 'SEARCH_ARUCO':
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge error: {e}')
            return

        # Detect markers
        corners, ids, _ = cv2.aruco.detectMarkers(
            frame, self.aruco_dict, parameters=self.aruco_params
        )

        if ids is None or len(ids) == 0:
            return

        # For simplicity: take first detected marker
        ids = ids.flatten()
        for i, marker_id in enumerate(ids):
            # Compute image center of the marker
            pts = corners[i][0]
            center_x = float(np.mean(pts[:, 0]))
            center_y = float(np.mean(pts[:, 1]))

            # Example hack: convert pixel offsets to a rough local position
            h, w, _ = frame.shape
            dx_pixels = center_x - w / 2.0
            dy_pixels = center_y - h / 2.0

            # Assume marker about 1 m in front, small-angle approx
            scale = 0.002  # tune this for your camera/FOV
            x = 1.0  # 1 m forward from camera
            y = -dx_pixels * scale
            z = 0.0

            # Store pose (still in camera / robot frame; good enough for assignment-level demo)
            self.marker_poses[marker_id] = (self.current_pose[0] + x,  # very rough
                                            self.current_pose[1] + y,
                                            z)

            # Report detection
            report = String()
            report.data = f'aruco{marker_id} position x:{self.marker_poses[marker_id][0]:.2f}, y:{self.marker_poses[marker_id][1]:.2f}, z:{z:.2f}'
            self.report_pub.publish(report)
            self.get_logger().info(f'Detected aruco{marker_id} at {self.marker_poses[marker_id]}')

            # Stop search after first detection
            self.stop_robot()
            self.state = 'STANDBY'
            self.publish_status('ready')
            break

    # ------------- FSM loop -------------

    def state_machine_step(self):
        if self.state == 'STANDBY':
            self.stop_robot()

        elif self.state == 'SEARCH_ARUCO':
            self.do_search_aruco()

        elif self.state == 'MOVE_TO_MARKER':
            self.do_move_to_marker()

        elif self.state == 'RETURN_TO_ORIGIN':
            self.do_return_to_origin()

    # ------------- State behaviors -------------

    def do_search_aruco(self):
        """Square + rotate pattern to check all walls"""
        # Initialize on first call
        if self.search_start_time is None:
            self.search_start_time = self.get_clock().now()
            self.square_phase = 0  # 0=forward, 1=turn90, 2=forward, 3=turn90+spin
            self.phase_start_time = self.get_clock().now()
            self.phase_distance = 0.0
            return
        
        # 60s timeout
        dt_total = (self.get_clock().now() - self.search_start_time).nanoseconds / 1e9
        if dt_total > 60.0:
            self.stop_robot()
            self.publish_status('search_timeout')
            self.state = 'STANDBY'
            return
        
        now = self.get_clock().now()
        dt_phase = (now - self.phase_start_time).nanoseconds / 1e9
        
        twist = Twist()
        
        if self.square_phase == 0 or self.square_phase == 2:  # Forward 1m
            if self.phase_distance < self.square_side:
                twist.linear.x = 0.3
                self.phase_distance += 0.3 * dt_phase
            else:
                # Side complete, turn 90°
                twist.angular.z = math.pi / 2  # 90° left
                if dt_phase > 2.0:  # 2s turn time
                    self.square_phase = (self.square_phase + 1) % 4
                    self.phase_start_time = now
                    self.phase_distance = 0.0
        
        elif self.square_phase == 1 or self.square_phase == 3:  # Rotate camera 360°
            twist.angular.z = 0.5  # Full spin to check walls
            if dt_phase > 6.0:  # 6s full rotation
                self.square_phase = (self.square_phase + 1) % 4
                self.phase_start_time = now
                self.phase_distance = 0.0
        
        self.cmd_pub.publish(twist)

    def do_move_to_marker(self):
        if self.current_goal_id not in self.marker_poses:
            self.publish_status('marker_unknown')
            self.stop_robot()
            return

        goal_x, goal_y, _ = self.marker_poses[self.current_goal_id]
        x, y, yaw = self.current_pose

        dx = goal_x - x
        dy = goal_y - y
        dist = math.hypot(dx, dy)
        heading = math.atan2(dy, dx)
        heading_error = normalize_angle(heading - yaw)

        # arrival check: Distance < 0.5m AND Heading Error < 10 deg
        if dist < 0.5 and abs(heading_error) < math.radians(10):
            self.stop_robot()
            msg = String()
            msg.data = f'arrived to {self.current_goal_id}'
            self.report_pub.publish(msg)
            return

        twist = Twist()
        twist.linear.x = min(0.4, 0.5 * dist)
        twist.angular.z = 1.5 * heading_error
        self.cmd_pub.publish(twist)

    def do_return_to_origin(self):
        if self.origin_pose is None:
            self.publish_status('no_origin_pose')
            self.stop_robot()
            return

        goal_x, goal_y, goal_yaw = self.origin_pose
        x, y, yaw = self.current_pose

        dx = goal_x - x
        dy = goal_y - y
        dist = math.hypot(dx, dy)
        heading = math.atan2(dy, dx)
        heading_error = normalize_angle(heading - yaw)

        if dist < 0.3 and abs(normalize_angle(yaw - goal_yaw)) < math.radians(10):
            self.stop_robot()
            report = String()
            report.data = 'arrived to origin'
            self.report_pub.publish(report)
            self.publish_status('ready')
            self.state = 'STANDBY'
            return

        twist = Twist()
        twist.linear.x = min(0.3, 0.5 * dist)
        twist.angular.z = 1.5 * heading_error
        self.cmd_pub.publish(twist)

    # ------------- Helpers -------------

    def stop_robot(self):
        twist = Twist()
        self.cmd_pub.publish(twist)

    def publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
