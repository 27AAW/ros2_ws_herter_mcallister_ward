#!/usr/bin/env python3

import math
from typing import Dict, Tuple
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo
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
        self.completed_missions = set()
        self.marker_poses: Dict[int, Tuple[float, float, float]] = {}
        self.current_pose = (0.0, 0.0, 0.0)  # x, y, yaw
        self.search_start_time = None
        self.origin_pose = None
        self.no_aruco_detected = False

        # Warehouse search variables
        self.search_phase_time = None
        self.spiral_radius = 0.0

        # INSPECTION variables
        self.inspection_state = 'IDLE'
        self.image1 = None
        self.image2 = None
        self.image1_time = None

        # OpenCV setup
        self.bridge = CvBridge()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.aruco_params.adaptiveThreshWinSizeMin = 3
        self.aruco_params.adaptiveThreshWinSizeMax = 23
        self.aruco_params.adaptiveThreshConstant = 7
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        # Camera calibration
        self.K = np.eye(3)
        self.dist_coeffs = np.zeros((5, 1))
        self.camera_matrix_ready = False

        # Subscribers - FIXED: /image_raw for simulator
        self.missions_sub = self.create_subscription(String, '/missions', self.missions_callback, 10)
        self.eval_sub = self.create_subscription(String, '/evaluator_message', self.evaluator_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.image1_sub = self.create_subscription(Image, '/image1', self.image1_callback, 10)
        self.image2_sub = self.create_subscription(Image, '/image2', self.image2_callback, 10)
        self.image_raw_sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)  # FIXED
        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera_info', self.camera_info_callback, 10)

        # Publishers
        self.status_pub = self.create_publisher(String, '/robot_status', 10)
        self.report_pub = self.create_publisher(String, '/robot_report', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # FSM timer
        self.timer = self.create_timer(0.1, self.state_machine_step)

        self.publish_status('ready')
        self.get_logger().info('MissionController started')

    def camera_info_callback(self, msg: CameraInfo):
        self.K = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)
        self.camera_matrix_ready = True
        self.get_logger().info('Camera calibration loaded')

    def missions_callback(self, msg: String):
        text = msg.data.strip()
        
        # FIXED #1: Spam protection FIRST
        if text == 'search_aruco' and self.state == 'SEARCH_ARUCO':
            return  # Ignore duplicates immediately
            
        self.get_logger().info(f'MISSION: "{text}" (state={self.state})')

        if text == 'search_aruco':
            self.state = 'SEARCH_ARUCO'
            self.search_start_time = self.get_clock().now()
            self.no_aruco_detected = False
            self.search_phase_time = self.get_clock().now()
            self.spiral_radius = 0.0
            self.publish_status('searching')

        elif text.startswith('move to'):
            parts = text.split()
            if len(parts) == 3 and parts[2].isdigit():
                goal_id = int(parts[2])
                if goal_id in self.completed_missions:
                    self.get_logger().info(f'Already completed marker {goal_id}')
                    return
                self.current_goal_id = goal_id
                self.state = 'MOVE_TO_MARKER'
                self.publish_status(f'moving_to_{self.current_goal_id}')
                self.get_logger().info(f'TARGET {self.current_goal_id}: {self.marker_poses.get(self.current_goal_id, "MISSING")}')
            else:
                self.get_logger().warn('Bad "move to" format')

        elif text == 'image_analysis':
            self.state = 'IMAGE_ANALYSIS'
            self.inspection_state = 'ANALYZING'
            self.image1 = None
            self.image2 = None
            self.publish_status('analyze image')

        elif text in ['return_to_origin', 'return to origin']:
            self.state = 'RETURN_TO_ORIGIN'
            self.publish_status('returning')

        elif text == 'standby':
            self.state = 'STANDBY'
            self.stop_robot()
            self.publish_status('ready')

        else:
            self.get_logger().warn(f'Unknown mission: {text}')

    def evaluator_callback(self, msg: String):
        text = msg.data.strip().lower()
        self.get_logger().info(f'EVALUATOR: "{text}"')
        if 'success' in text and self.state == 'MOVE_TO_MARKER':
            self.state = 'RETURN_TO_ORIGIN'
            self.publish_status('returning')

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
        # FIXED #2: Aggressive debug logging
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            self.get_logger().info(f"IMAGE: {frame.shape} brightness={np.mean(gray):.1f}")  # Debug
            
            if self.state != 'SEARCH_ARUCO':
                return

            corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

            if ids is not None and len(ids) > 0:
                self.get_logger().info(f'ARUCO DETECTED: {len(ids)} markers!')
                ids = ids.flatten()
                h, w, _ = frame.shape
                for i, marker_id in enumerate(ids):
                    pts = corners[i][0]
                    center_x = float(np.mean(pts[:, 0]))
                    center_y = float(np.mean(pts[:, 1]))

                    marker_size = 0.05
                    if self.camera_matrix_ready:
                        corners_3d = np.array([[0, 0, 0], [marker_size, 0, 0], 
                                               [marker_size, marker_size, 0], [0, marker_size, 0]], dtype=np.float32)
                        success, rvec, tvec = cv2.solvePnP(corners_3d, pts, self.K, self.dist_coeffs)
                        if success:
                            x_offset = tvec[0][0]
                            y_offset = tvec[1][0]
                            self.marker_poses[marker_id] = (self.current_pose[0] + x_offset,
                                                            self.current_pose[1] + y_offset, 0.0)
                            self.get_logger().info(f'3D ARUCO {marker_id}: ({x_offset:.2f}m, {y_offset:.2f}m)')
                            continue

                    # Fallback 2D estimation
                    report = String()
                    report.data = f"arucoin position {center_x:.1f}, {center_y:.1f}, 0.5"
                    self.report_pub.publish(report)
                    dx_pixels = center_x - w / 2.0
                    scale = 0.002
                    x_offset = 1.0
                    y_offset = -dx_pixels * scale
                    self.marker_poses[marker_id] = (self.current_pose[0] + x_offset, 
                                                    self.current_pose[1] + y_offset, 0.0)
                    self.no_aruco_detected = True
                    self.get_logger().info(f'ARUCO {marker_id} -> world: {self.marker_poses[marker_id]}')
        except Exception as e:
            self.get_logger().warn(f'Image processing error: {e}')

    def image1_callback(self, msg: Image):
        if self.inspection_state == 'WAITING_IMAGE1':
            self.image1 = msg
            self.image1_time = self.get_clock().now()
            self.inspection_state = 'WAITING_IMAGE2'
            self.get_logger().info('Received image1, waiting for image2')

    def image2_callback(self, msg: Image):
        if self.inspection_state == 'WAITING_IMAGE2' and self.image1 is not None:
            self.image2 = msg
            self.analyze_movement()
            self.inspection_state = 'IDLE'

    def analyze_movement(self):
        try:
            img1 = self.bridge.imgmsg_to_cv2(self.image1, "bgr8")
            img2 = self.bridge.imgmsg_to_cv2(self.image2, "bgr8")
            gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
            gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

            diff = cv2.absdiff(gray1, gray2)
            thresh = cv2.threshold(diff, 25, 255, cv2.THRESH_BINARY)[1]
            thresh = cv2.dilate(thresh, None, iterations=2)

            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    report = String()
                    report.data = f"movement at x: {cx}, y: {cy}"
                    self.report_pub.publish(report)
                    self.get_logger().info(f'Movement detected at ({cx}, {cy})')
                else:
                    self.get_logger().warn('No valid movement moments')
            else:
                self.get_logger().warn('No movement contours found')

        except Exception as e:
            self.get_logger().error(f'Movement analysis error: {e}')

    def state_machine_step(self):
        """FIXED #3: Simple if-elif chain, NO early returns"""
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

    def do_search_aruco(self):
        if self.search_phase_time is None or self.search_start_time is None:
            self.search_phase_time = self.get_clock().now()
            self.search_start_time = self.get_clock().now()
            return

        dt_total = (self.get_clock().now() - self.search_start_time).nanoseconds / 1e9
        if dt_total > 180.0:
            self.stop_robot()
            if not self.no_aruco_detected:
                self.publish_status('no_arucos_detected')
            self.state = 'STANDBY'
            return

        now = self.get_clock().now()
        dt_since_phase_start = (now - self.search_phase_time).nanoseconds / 1e9
        twist = Twist()

        WALL_GLANCE_INTERVAL = 6.0
        GLANCE_TURN_TIME = 0.8
        SPIRAL_FORWARD_TIME = 8.0
        OSCILLATION_FREQ = 2.0

        glance_cycle = int(dt_since_phase_start / WALL_GLANCE_INTERVAL) % 2
        glance_phase = dt_since_phase_start % WALL_GLANCE_INTERVAL

        if glance_cycle == 0:  # WALL GLANCE
            if glance_phase < GLANCE_TURN_TIME:
                twist.angular.z = 1.0
            elif glance_phase < GLANCE_TURN_TIME * 2:
                twist.angular.z = -1.0
            else:
                twist.linear.x = 0.25
                twist.angular.z = 0.2 * math.sin(glance_phase * OSCILLATION_FREQ)
        else:  # SPIRAL
            spiral_time = dt_since_phase_start % SPIRAL_FORWARD_TIME
            if spiral_time < SPIRAL_FORWARD_TIME * 0.8:
                twist.linear.x = 0.3
                spiral_turn = 0.15 + (self.spiral_radius * 0.05)
                weave = 0.25 * math.sin(spiral_time * OSCILLATION_FREQ)
                twist.angular.z = spiral_turn + weave
            else:
                twist.angular.z = 0.4
                if spiral_time > SPIRAL_FORWARD_TIME:
                    self.spiral_radius += 0.1
                    self.search_phase_time = now

        if self.spiral_radius > 20:
            self.spiral_radius = 0.0

        self.cmd_pub.publish(twist)

    def do_move_to_marker(self):
        if self.current_goal_id not in self.marker_poses:
            self.get_logger().error(f'Marker {self.current_goal_id} not found!')
            self.state = 'STANDBY'
            return

        goal_x, goal_y, _ = self.marker_poses[self.current_goal_id]
        x, y, yaw = self.current_pose
        dx = goal_x - x
        dy = goal_y - y
        dist = math.hypot(dx, dy)
        heading = math.atan2(dy, dx)
        heading_error = normalize_angle(heading - yaw)

        self.get_logger().info(f'NAV: dist={dist:.2f}m, heading_err={math.degrees(heading_error):.1f}°')

        if dist < 0.5 and abs(heading_error) < math.radians(10):
            self.stop_robot()
            self.completed_missions.add(self.current_goal_id)
            msg = String()
            msg.data = f'arrived to {self.current_goal_id}'
            self.report_pub.publish(msg)
            self.get_logger().info(f'ARRIVED TO MARKER {self.current_goal_id}')
            self.current_goal_id = None
            self.state = 'STANDBY'
            self.publish_status('ready')
            return

        twist = Twist()
        twist.linear.x = min(0.4, 0.5 * dist)
        twist.angular.z = 1.5 * heading_error
        self.cmd_pub.publish(twist)

    def do_return_to_origin(self):
        if self.origin_pose is None:
            self.get_logger().warn('No origin pose!')
            self.state = 'STANDBY'
            return

        goal_x, goal_y, _ = self.origin_pose
        x, y, yaw = self.current_pose
        dx = goal_x - x
        dy = goal_y - y
        dist = math.hypot(dx, dy)

        if dist < 0.2:
            self.stop_robot()
            report = String()
            report.data = 'arrived to origin'
            self.report_pub.publish(report)
            self.get_logger().info('ARRIVED TO ORIGIN')
            self.state = 'STANDBY'
            self.publish_status('ready')
            return

        heading = math.atan2(dy, dx)
        heading_error = normalize_angle(heading - yaw)
        twist = Twist()
        twist.linear.x = min(0.3, 0.5 * dist)
        twist.angular.z = 1.5 * heading_error
        self.cmd_pub.publish(twist)

    def do_image_analysis(self):
        if self.inspection_state == 'ANALYZING':
            self.inspection_state = 'WAITING_IMAGE1'
            self.image1_time = self.get_clock().now()
            return
        
        elif self.inspection_state == 'WAITING_IMAGE1':
            dt_wait = (self.get_clock().now() - self.image1_time).nanoseconds / 1e9
            if dt_wait > 3.0:  # Timeout
                report = String()
                report.data = "no movement detected"
                self.report_pub.publish(report)
                self.state = 'STANDBY'
                self.inspection_state = 'IDLE'
                self.publish_status('ready')

        self.stop_robot()

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
