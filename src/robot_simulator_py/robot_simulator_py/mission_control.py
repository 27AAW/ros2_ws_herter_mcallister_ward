import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time
import math

class MissionControlNode(Node):
    def __init__(self):
        super().__init__('mission_control')
        self.get_logger().info("Mission Control Node started")

        # Publishers
        self.status_pub = self.create_publisher(String, '/robot_status', 10)
        self.report_pub = self.create_publisher(String, '/robot_report', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers
        self.mission_sub = self.create_subscription(String, '/missions', self.mission_callback, 10)
        self.evaluator_sub = self.create_subscription(String, '/evaluator_message', self.evaluator_callback, 10)

        # State variables
        self.current_mission = None
        self.state = 'standby'
        self.start_time = None
        
        # Publish ready status on startup
        self.status_pub.publish(String(data="ready"))
        self.get_logger().info("Published 'ready' status - waiting for missions...")

        # For simulation purposes
        self.aruco_positions = {
            1: (1.0, 2.0, 0.0),
            2: (-1.0, 3.0, 0.0),
            3: (2.0, -1.0, 0.0),
            4: (-2.0, -2.0, 0.0)
        }
        self.current_position = (0.0, 0.0, 0.0)  # x, y, theta

    def mission_callback(self, msg):
        mission = msg.data
        self.get_logger().info(f"Received mission: {mission}")
        self.current_mission = mission
        self.execute_mission()

    def evaluator_callback(self, msg):
        feedback = msg.data
        self.get_logger().info(f"Evaluator feedback: {feedback}")

    def execute_mission(self):
        if self.current_mission == 'search_aruco':
            self.search_aruco()
        elif self.current_mission.startswith('move to'):
            marker_id = int(self.current_mission.split()[-1])
            self.move_to_marker(marker_id)
        elif self.current_mission == 'return to origin':
            self.return_to_origin()
        elif self.current_mission == 'image_analysis':
            self.image_analysis()
        elif self.current_mission == 'image_analysis2':
            self.image_analysis2()
        else:
            self.get_logger().warn(f"Unknown mission: {self.current_mission}")

    def search_aruco(self):
        self.get_logger().info("Executing search_aruco mission - systematic area search")
        self.status_pub.publish(String(data="searching"))
        self.start_time = time.time()
        
        # Define search area and pattern
        search_area_size = 8.0  # 8m x 8m area
        step_size = 1.0  # 1m steps
        search_timeout = 60.0  # 60 second limit
        
        # Generate spiral search waypoints
        waypoints = self.generate_spiral_waypoints(search_area_size, step_size)
        self.get_logger().info(f"Generated {len(waypoints)} search waypoints")
        
        found_markers = []
        
        for i, (target_x, target_y) in enumerate(waypoints):
            # Check timeout
            if time.time() - self.start_time > search_timeout:
                self.get_logger().warn("Search timeout reached")
                break
                
            self.get_logger().info(f"Moving to search waypoint {i+1}/{len(waypoints)}: ({target_x:.2f}, {target_y:.2f})")
            
            # Move to waypoint
            self.move_to_position(target_x, target_y)
            
            # Pause briefly to "look around" at each waypoint
            time.sleep(0.5)
            
            # Simulate potential marker detection (in real implementation, this would come from camera)
            # For now, we'll simulate finding markers at certain positions
            detected_markers = self.simulate_marker_detection(target_x, target_y)
            for marker in detected_markers:
                if marker not in found_markers:
                    found_markers.append(marker)
                    marker_id, x, y, z = marker
                    self.report_pub.publish(String(data=f"aruco {marker_id} in position x: {x}, y: {y}, z: {z}"))
                    self.get_logger().info(f"Found ArUco marker {marker_id} at ({x}, {y}, {z})")
        
        # Stop movement
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        
        if found_markers:
            self.get_logger().info(f"Search completed - found {len(found_markers)} markers")
        else:
            self.get_logger().info("Search completed - no markers found")
            
        self.status_pub.publish(String(data="ready"))

    def generate_spiral_waypoints(self, area_size, step_size):
        """Generate waypoints in a spiral pattern covering the search area"""
        waypoints = []
        center_x, center_y = 0.0, 0.0  # Start from origin
        
        # Spiral parameters
        max_radius = area_size / 2.0
        angle_step = 0.5  # radians between waypoints
        radius_step = step_size
        
        radius = 0.0
        angle = 0.0
        
        while radius <= max_radius:
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            waypoints.append((x, y))
            
            angle += angle_step
            # Increase radius gradually to create spiral
            radius = min(radius + radius_step * (angle_step / (2 * math.pi)), max_radius)
        
        # Add some edge points to ensure full coverage
        for r in [max_radius * 0.7, max_radius * 0.9, max_radius]:
            for angle in [0, math.pi/2, math.pi, 3*math.pi/2]:
                x = center_x + r * math.cos(angle)
                y = center_y + r * math.sin(angle)
                waypoints.append((x, y))
        
        return waypoints

    def move_to_position(self, target_x, target_y):
        """Move robot to a specific position"""
        # Calculate distance and angle to target
        current_x, current_y, current_theta = self.current_position
        distance = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
        target_angle = math.atan2(target_y - current_y, target_x - current_x)
        
        # Normalize angle difference
        angle_diff = target_angle - current_theta
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # Turn to face target
        twist = Twist()
        if abs(angle_diff) > 0.1:  # Only turn if significant angle difference
            turn_direction = 1.0 if angle_diff > 0 else -1.0
            twist.angular.z = turn_direction * 0.5
            self.cmd_vel_pub.publish(twist)
            time.sleep(abs(angle_diff) / 0.5)  # Wait for turn to complete
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
        
        # Move forward
        if distance > 0.1:  # Only move if significant distance
            twist.linear.x = 0.3  # Slower speed for precision
            self.cmd_vel_pub.publish(twist)
            time.sleep(distance / 0.3)  # Wait for movement to complete
            twist.linear.x = 0.0
            self.cmd_vel_pub.publish(twist)
        
        # Update current position
        self.current_position = (target_x, target_y, target_angle)

    def simulate_marker_detection(self, current_x, current_y):
        """Simulate marker detection at current position"""
        detected = []
        detection_range = 2.0  # Can detect markers within 2m
        
        for marker_id, (marker_x, marker_y, marker_z) in self.aruco_positions.items():
            distance = math.sqrt((marker_x - current_x)**2 + (marker_y - current_y)**2)
            if distance <= detection_range:
                detected.append((marker_id, marker_x, marker_y, marker_z))
        
        return detected

    def move_to_marker(self, marker_id):
        self.get_logger().info(f"Moving to marker {marker_id}")
        self.status_pub.publish(String(data="moving"))

        # Simulate movement to marker
        target_x, target_y, target_z = self.aruco_positions[marker_id]

        # Simple movement simulation: assume we can move directly
        distance = math.sqrt((target_x - self.current_position[0])**2 + (target_y - self.current_position[1])**2)
        angle = math.atan2(target_y - self.current_position[1], target_x - self.current_position[0])

        # Turn to face the target
        twist = Twist()
        twist.angular.z = 0.5 if angle > 0 else -0.5
        self.cmd_vel_pub.publish(twist)
        time.sleep(abs(angle) / 0.5)  # Simulate turn time
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

        # Move forward
        twist.linear.x = 0.5
        self.cmd_vel_pub.publish(twist)
        time.sleep(distance / 0.5)  # Simulate move time
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)

        # Update position
        self.current_position = (target_x, target_y, angle)

        self.report_pub.publish(String(data=f"arrived to {marker_id}"))
        self.status_pub.publish(String(data="ready"))

    def return_to_origin(self):
        self.get_logger().info("Returning to origin")
        self.status_pub.publish(String(data="returning"))

        # Simulate return to origin
        distance = math.sqrt(self.current_position[0]**2 + self.current_position[1]**2)
        angle = math.atan2(-self.current_position[1], -self.current_position[0])

        # Turn to face origin
        twist = Twist()
        twist.angular.z = 0.5 if angle > 0 else -0.5
        self.cmd_vel_pub.publish(twist)
        time.sleep(abs(angle) / 0.5)
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

        # Move to origin
        twist.linear.x = 0.5
        self.cmd_vel_pub.publish(twist)
        time.sleep(distance / 0.5)
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)

        self.current_position = (0.0, 0.0, 0.0)

        self.report_pub.publish(String(data="arrived to origin"))
        self.status_pub.publish(String(data="ready"))

    def image_analysis(self):
        self.get_logger().info("Performing image analysis")
        self.status_pub.publish(String(data="analyze image"))

        # Simulate image analysis
        time.sleep(1)
        # Simulate movement detection at some position
        x, y = 1.0, 2.0
        self.report_pub.publish(String(data=f"movement at x: {x}, y: {y}"))
        self.status_pub.publish(String(data="ready"))

    def image_analysis2(self):
        self.get_logger().info("Performing feature counting")
        self.status_pub.publish(String(data="analyze image2"))

        # Simulate feature counting
        time.sleep(1)
        num_features = 5  # Simulate
        self.report_pub.publish(String(data=f"{num_features} features detected"))
        self.status_pub.publish(String(data="ready"))

def main(args=None):
    rclpy.init(args=args)
    node = MissionControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()