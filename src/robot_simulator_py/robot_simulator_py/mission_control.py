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
        self.state = 'STANDBY'
        self.current_goal_id = None
        self.marker_poses: Dict[int, Tuple[float, float, float]] = {}
        self.current_pose = (0.0, 0.0, 0.0)  # x, y, yaw
        self.search_start_time = None
        self.origin_pose = None
        self.no_aruco_detected = False

        # Square search variables
        self.square_phase = 0
        self.phase_start_time = None
        self.phase_distance = 0.0
        self.square_side = 1.0
        self.phase_start_x = 0.0

        # OpenCV / camera
        self.bridge = CvBridge()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        
        # Image analysis variables
        self.image1_data = None
        self.image2_data = None
        self.image_analysis_done = False

        # Subscribers
        self.missions_sub = self.create_subscription(String, '/missions', self.missions_callback, 10)
        self.eval_sub = self.create_subscription(String, '/evaluator_message', self.evaluator_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.image1_sub = self.create_subscription(Image, 'image1', self.image1_callback, 10)
        self.image2_sub = self.create_subscription(Image, 'image2', self.image2_callback, 10)

        # Publishers
        self.status_pub = self.create_publisher(String, '/robot_status', 10)
        self.report_pub = self.create_publisher(String, '/robot_report', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # FSM timer
        self.timer = self.create_timer(0.1, self.state_machine_step)

        # For simulation
        self.aruco_positions = {
            1: (1.0, 2.0, 0.0),
            2: (-1.0, 3.0, 0.0),
            3: (2.0, -1.0, 0.0),
            4: (-2.0, -2.0, 0.0)
        }

        # Initialization
        self.publish_status('ready')
        self.get_logger().info('MissionController started, state=STANDBY')

    def image1_callback(self, msg):
        self.image1_data = msg

    def image2_callback(self, msg):
        self.image2_data = msg

    def missions_callback(self, msg: String):
        text = msg.data.strip()
        
        # FIXED: IGNORE DUPLICATE COMMANDS
        if self.state == 'SEARCH_ARUCO' and text == 'search_aruco':
            return  # Already searching!
        if self.state == 'STANDBY' and text == 'standby':
            return  # Already standby!
            
        self.get_logger().info(f'/missions: "{text}"')

        if text == 'search_aruco':
            self.state = 'SEARCH_ARUCO'
            self.search_start_time = self.get_clock().now()
            self.no_aruco_detected = False
            self.phase_start_time = self.get_clock().now()
            self.square_phase = 0
            self.publish_status('searching')

        elif text.startswith('move to'):
            parts = text.split()
            if len(parts) == 3 and parts[2].isdigit():
                self.current_goal_id = int(parts[2])
                self.state = 'MOVE_TO_MARKER'
                self.publish_status(f'moving_to_{self.current_goal_id}')
            else:
                self.get_logger().warn('Bad "move to" mission format')

        elif text in ['return_to_origin', 'return to origin']:
            self.state = 'RETURN_TO_ORIGIN'
            self.publish_status('returning')

        elif text == 'image_analysis':
            self.state = 'IMAGE_ANALYSIS'
            self.image_analysis_done = False
            self.publish_status('analyze image')

        elif text == 'image_analysis2':
            self.state = 'FEATURE_COUNTING'
            self.image_analysis_done = False
            self.publish_status('analyze image2')

        elif text == 'standby':
            self.state = 'STANDBY'
            self.stop_robot()
            self.publish_status('ready')
            self.get_logger().info('Entered STANDBY mode')

        else:
            self.get_logger().warn(f'Unknown mission: {text}')

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

        if self.origin_pose is None:
            self.origin_pose = (x, y, yaw)

    def image_callback(self, msg: Image):
        if self.state != 'SEARCH_ARUCO':
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge error: {e}')
            return

        corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)

        if ids is None or len(ids) == 0:
            return

        ids = ids.flatten()
        for i, marker_id in enumerate(ids):
            pts = corners[i][0]
            center_x = float(np.mean(pts[:, 0]))
            center_y = float(np.mean(pts[:, 1]))

            h, w, _ = frame.shape
            dx_pixels = center_x - w / 2.0
            dy_pixels = center_y - h / 2.0

            scale = 0.002
            x = 1.0
            y = -dx_pixels * scale
            z = 0.0

            self.marker_poses[marker_id] = (self.current_pose[0] + x, self.current_pose[1] + y, z)
            self.no_aruco_detected = True

            report = String()
            report.data = f'aruco{marker_id} position x:{self.marker_poses[marker_id][0]:.2f}, y:{self.marker_poses[marker_id][1]:.2f}, z:{z:.2f}'
            self.report_pub.publish(report)
            self.get_logger().info(f'Detected aruco{marker_id} at {self.marker_poses[marker_id]}')

            self.stop_robot()
            self.state = 'STANDBY'
            self.publish_status('ready')
            break

    def state_machine_step(self):
        if self.state == 'STANDBY':
            self.stop_robot()

        elif self.state == 'SEARCH_ARUCO':
            self.do_search_aruco()

        elif self.state == 'MOVE_TO_MARKER':
            self.do_move_to_marker()

        elif self.state == 'RETURN_TO_ORIGIN':
            self.do_return_to_origin()

        elif self.state == 'IMAGE_ANALYSIS':
            self.do_image_analysis()

        elif self.state == 'FEATURE_COUNTING':
            self.do_feature_counting()

    def do_image_analysis(self):
        if self.image1_data is None or self.image2_data is None or self.image_analysis_done:
            return

        try:
            frame1 = self.bridge.imgmsg_to_cv2(self.image1_data, desired_encoding='bgr8')
            frame2 = self.bridge.imgmsg_to_cv2(self.image2_data, desired_encoding='bgr8')
            
            gray1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
            gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
            
            diff = cv2.absdiff(gray1, gray2)
            _, thresh = cv2.threshold(diff, 30, 255, cv2.THRESH_BINARY)
            
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    
                    report = String()
                    report.data = f"movement at x:{cx}, y:{cy}"
                    self.report_pub.publish(report)
                    self.get_logger().info(f'Motion detected at x:{cx}, y:{cy}')
            
            self.image_analysis_done = True
            self.state = 'STANDBY'
            self.publish_status('ready')
            
        except Exception as e:
            self.get_logger().error(f'Image analysis error: {e}')

    def do_feature_counting(self):
        if self.image1_data is None or self.image2_data is None or self.image_analysis_done:
            return

        try:
            frame1 = self.bridge.imgmsg_to_cv2(self.image1_data, desired_encoding='bgr8')
            frame2 = self.bridge.imgmsg_to_cv2(self.image2_data, desired_encoding='bgr8')
            
            gray1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
            gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
            
            diff = cv2.absdiff(gray1, gray2)
            _, thresh = cv2.threshold(diff, 30, 255, cv2.THRESH_BINARY)
            
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            feature_count = len([c for c in contours if cv2.contourArea(c) > 100])
            
            report = String()
            report.data = f"{feature_count} features detected"
            self.report_pub.publish(report)
            self.get_logger().info(f'{feature_count} features detected')
            
            self.image_analysis_done = True
            self.state = 'STANDBY'
            self.publish_status('ready')
            
        except Exception as e:
            self.get_logger().error(f'Feature counting error: {e}')

    def do_search_aruco(self):
        """Square pattern - FIXED phase transitions"""
        if self.phase_start_time is None or self.search_start_time is None:
            self.phase_start_time = self.get_clock().now()
            self.search_start_time = self.get_clock().now()
            self.square_phase = 0
            return

        dt_total = (self.get_clock().now() - self.search_start_time).nanoseconds / 1e9
        if dt_total > 60.0:
            self.stop_robot()
            if not self.no_aruco_detected:
                self.publish_status('no_arucos_detected')
                self.get_logger().warn('Search complete: NO ARUCOS DETECTED')
            else:
                self.publish_status('search_timeout')
            self.state = 'STANDBY'
            return

        now = self.get_clock().now()
        dt_since_phase_start = (now - self.phase_start_time).nanoseconds / 1e9

        twist = Twist()

        if self.square_phase == 0 or self.square_phase == 2:  # Forward 3s
            if dt_since_phase_start < 3.0:
                twist.linear.x = 0.3
            else:
                self.square_phase = (self.square_phase + 1) % 4
                self.phase_start_time = now
                self.get_logger().info(f'Phase advance: {self.square_phase-1} -> {self.square_phase}')

        elif self.square_phase == 1 or self.square_phase == 3:  # Spin 6s
            twist.angular.z = 0.5
            if dt_since_phase_start > 6.0:
                self.square_phase = (self.square_phase + 1) % 4
                self.phase_start_time = now
                self.get_logger().info(f'Phase advance: {self.square_phase-1} -> {self.square_phase}')

        self.cmd_pub.publish(twist)
        self.get_logger().info(f'phase={self.square_phase}, dt={dt_since_phase_start:.1f}s, vel={twist.linear.x:.2f}/{twist.angular.z:.2f}')

    def do_move_to_marker(self):
        if self.current_goal_id not in self.marker_poses and self.current_goal_id not in self.aruco_positions:
            self.publish_status('marker_unknown')
            self.stop_robot()
            self.state = 'STANDBY'
            return

        goal_x, goal_y, _ = self.marker_poses.get(self.current_goal_id, self.aruco_positions.get(self.current_goal_id, (0,0,0)))
        x, y, yaw = self.current_pose

        dx = goal_x - x
        dy = goal_y - y
        dist = math.hypot(dx, dy)
        heading = math.atan2(dy, dx)
        heading_error = normalize_angle(heading - yaw)

        if dist < 0.5 and abs(heading_error) < math.radians(10):
            self.stop_robot()
            msg = String()
            msg.data = f'arrived to {self.current_goal_id}'
            self.report_pub.publish(msg)
            self.state = 'STANDBY'
            self.publish_status('ready')
            return

        twist = Twist()
        twist.linear.x = min(0.4, 0.5 * dist)
        twist.angular.z = 1.5 * heading_error
        self.cmd_pub.publish(twist)

    def do_return_to_origin(self):
        if self.origin_pose is None:
            self.publish_status('no_origin_pose')
            self.stop_robot()
            self.state = 'STANDBY'
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
