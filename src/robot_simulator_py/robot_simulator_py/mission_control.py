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

        # OpenCV setup - FIXED: Create ArucoDetector
        self.bridge = CvBridge()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.aruco_params.adaptiveThreshWinSizeMin = 3
        self.aruco_params.adaptiveThreshWinSizeMax = 23
        self.aruco_params.adaptiveThreshConstant = 7
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)  # NEW LINE

        # Camera calibration
        self.K = np.eye(3)
        self.dist_coeffs = np.zeros((5, 1))
        self.camera_matrix_ready = False

        # Subscribers
        self.missions_sub = self.create_subscription(String, '/missions', self.missions_callback, 10)
        self.eval_sub = self.create_subscription(String, '/evaluator_message', self.evaluator_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.image1_sub = self.create_subscription(Image, '/image1', self.image1_callback, 10)
        self.image2_sub = self.create_subscription(Image, '/image2', self.image2_callback, 10)
        self.image_raw_sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera_info', self.camera_info_callback, 10)

        # Publishers
        self.status_pub = self.create_publisher(String, '/robot_status', 10)
        self.report_pub = self.create_publisher(String, '/robot_report', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # FSM timer
        self.timer = self.create_timer(0.1, self.state_machine_step)

        self.publish_status('ready')
        self.get_logger().info('MissionController started')

    # [All other callbacks unchanged until image_callback...]

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            self.get_logger().info(f"IMAGE: {frame.shape} brightness={np.mean(gray):.1f}")
            
            if self.state != 'SEARCH_ARUCO':
                return

            # FIXED: Use ArucoDetector instead of deprecated detectMarkers
            corners, ids, rejected = self.detector.detectMarkers(gray)  # CHANGED LINE

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

    # [Rest of the code unchanged - all other methods remain exactly the same]
    # ... [image1_callback, image2_callback, analyze_movement, state_machine_step, etc. remain identical]

    def camera_info_callback(self, msg: CameraInfo):
        self.K = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)
        self.camera_matrix_ready = True
        self.get_logger().info('Camera calibration loaded')

    def missions_callback(self, msg: String):
        text = msg.data.strip()
        
        if text == 'search_aruco' and self.state == 'SEARCH_ARUCO':
            return
        
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

    # [Include all other methods exactly as they were - I'm truncating for brevity]
    # The rest (evaluator_callback, odom_callback, analyze_movement, all do_* methods, etc.) 
    # remain completely unchanged

def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
