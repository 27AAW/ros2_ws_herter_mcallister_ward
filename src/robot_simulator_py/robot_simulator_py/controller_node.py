
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration

# Replace tf_transformations with custom function
def euler_from_quaternion(quat):
    x, y, z, w = quat
    roll = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch = math.asin(2*(w*y - z*x))
    yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return roll, pitch, yaw

def normalize_angle(angle):
    """Normalize angle to [-pi, pi]"""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.get_logger().info("Controller node starting...")

        # Publisher for /cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Publisher to /cmd_vel initialized")

        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.get_logger().info("TF listener initialized")

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Control loop timer started")

        # State machine variables
        self.state = 'FORWARD'  # States: FORWARD, TURN
        self.corner_count = 0   # Track which corner we're at (0-3)
        
        # Square parameters
        self.side_length = 2.0  # 2 meters per side
        self.target_angle = 0.0 # Target orientation after turn
        
        # Starting position (set on first successful transform)
        self.start_x = None
        self.start_y = None
        self.start_yaw = None
        
        # Current waypoint target
        self.target_x = None
        self.target_y = None
        
        # Control parameters
        self.max_linear_speed = 1.0     # m/s (maximum)
        self.max_angular_speed = 1.0     # rad/s (maximum)
        self.position_tolerance = 0.02   # meters
        self.angle_tolerance = 0.02      # radians (~1.15 degrees)
        
        # Proportional control gains
        self.kp_linear = 0.75   # Proportional gain for linear velocity
        self.kp_angular = 2.0  # Proportional gain for angular velocity
        
        # Minimum speeds to overcome friction
        self.min_linear_speed = 0.05   # m/s
        self.min_angular_speed = 0.1   # rad/s
        
        self.get_logger().info("Square navigation initialized: 2m x 2m square")

    def control_loop(self):
        try:
            # Get current transform
            trans = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            
            # Extract current pose
            current_x = trans.transform.translation.x
            current_y = trans.transform.translation.y
            q = trans.transform.rotation
            roll, pitch, current_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            
            # Initialize starting position on first run
            if self.start_x is None:
                self.start_x = current_x
                self.start_y = current_y
                self.start_yaw = current_yaw
                self.target_angle = current_yaw
                self.set_next_target()
                self.get_logger().info(f"Starting position: x={self.start_x:.2f}, y={self.start_y:.2f}, yaw={self.start_yaw:.2f}")
            
            # State machine logic
            twist = Twist()
            
            if self.state == 'FORWARD':
                # Calculate distance to target
                dx = self.target_x - current_x
                dy = self.target_y - current_y
                distance = math.sqrt(dx**2 + dy**2)
                
                # Check if we've reached the target position
                if distance < self.position_tolerance:
                    self.get_logger().info(f"Reached corner {self.corner_count}! Switching to TURN state")
                    self.state = 'TURN'
                    # Calculate target angle for next side (90 degree turn)
                    self.target_angle = normalize_angle(self.target_angle + math.pi / 2)
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                else:
                    # Proportional control for linear velocity
                    # Speed is proportional to distance, but clamped to max speed
                    linear_vel = self.kp_linear * distance
                    linear_vel = max(self.min_linear_speed, min(linear_vel, self.max_linear_speed))
                    
                    # Also add heading correction while driving forward
                    desired_yaw = math.atan2(dy, dx)
                    heading_error = normalize_angle(desired_yaw - current_yaw)
                    angular_vel = self.kp_angular * heading_error
                    angular_vel = max(-self.max_angular_speed, min(angular_vel, self.max_angular_speed))
                    
                    twist.linear.x = linear_vel
                    twist.angular.z = angular_vel
                    
                    self.get_logger().info(
                        f"FORWARD (corner {self.corner_count}): pos=({current_x:.3f}, {current_y:.3f}), "
                        f"target=({self.target_x:.3f}, {self.target_y:.3f}), dist={distance:.3f}m, "
                        f"v_lin={linear_vel:.2f}, v_ang={angular_vel:.2f}"
                    )
            
            elif self.state == 'TURN':
                # Calculate angle error
                angle_error = normalize_angle(self.target_angle - current_yaw)
                
                # Check if we've completed the turn
                if abs(angle_error) < self.angle_tolerance:
                    self.get_logger().info(f"Turn complete! Angle error: {angle_error:.4f} rad")
                    self.corner_count += 1
                    
                    # Check if we've completed the square
                    if self.corner_count >= 4:
                        self.get_logger().info("=== SQUARE COMPLETE! Returning to start ===")
                        self.corner_count = 0
                        # Reset to starting orientation for next lap
                        self.target_angle = self.start_yaw
                    
                    # Set next target and switch to forward
                    self.set_next_target()
                    self.state = 'FORWARD'
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                else:
                    # Proportional control for angular velocity
                    # Speed is proportional to angle error, but clamped to max speed
                    angular_vel = self.kp_angular * angle_error
                    
                    # Clamp to max angular speed
                    angular_vel = max(-self.max_angular_speed, min(angular_vel, self.max_angular_speed))
                    
                    # Apply minimum speed to overcome static friction (maintain sign)
                    if abs(angular_vel) < self.min_angular_speed:
                        angular_vel = self.min_angular_speed if angular_vel > 0 else -self.min_angular_speed
                    
                    twist.linear.x = 0.0
                    twist.angular.z = angular_vel
                    
                    self.get_logger().info(
                        f"TURN (corner {self.corner_count}): current_yaw={current_yaw:.3f}, "
                        f"target_yaw={self.target_angle:.3f}, error={angle_error:.3f}, v_ang={angular_vel:.2f}"
                    )
            
            # Publish velocity command
            self.cmd_pub.publish(twist)

        except Exception as e:
            self.get_logger().warn(f"Could not get transform: {e}")
    
    def set_next_target(self):
        """Calculate the next target position based on corner count"""
        # Square corners relative to start position (counterclockwise)
        # Corner 0: (2, 0) - East
        # Corner 1: (2, 2) - North-East  
        # Corner 2: (0, 2) - North
        # Corner 3: (0, 0) - Start
        
        if self.corner_count == 0:
            # First side: move East (positive X)
            self.target_x = self.start_x + self.side_length
            self.target_y = self.start_y
        elif self.corner_count == 1:
            # Second side: move North (positive Y)
            self.target_x = self.start_x + self.side_length
            self.target_y = self.start_y + self.side_length
        elif self.corner_count == 2:
            # Third side: move West (negative X)
            self.target_x = self.start_x
            self.target_y = self.start_y + self.side_length
        elif self.corner_count == 3:
            # Fourth side: move South (negative Y) - back to start
            self.target_x = self.start_x
            self.target_y = self.start_y
        
        self.get_logger().info(f"New target set: ({self.target_x:.2f}, {self.target_y:.2f})")

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
