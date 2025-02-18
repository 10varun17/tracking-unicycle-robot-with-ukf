#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Twist
import math

class PDController(Node):
    def __init__(self):
        super().__init__("pd_controller")
        self.declare_pd_parameters()
        
        self.target_x = self.get_parameter("target_x").value
        self.target_y = self.get_parameter("target_y").value
        self.kp_linear = self.get_parameter("kp_linear").value
        self.kd_linear = self.get_parameter("kd_linear").value
        self.kp_angular = self.get_parameter("kp_angular").value
        self.kd_angular = self.get_parameter("kd_angular").value
        self.v_max = self.get_parameter("v_max").value
        self.omega_max = self.get_parameter("omega_max").value
        
        self.prev_error_linear = 0.0
        self.prev_error_angular = 0.0
        self.prev_time = self.get_clock().now()
        
        # Create a publisher for control commands
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        
        # Create a subscriber to the estimated pose
        self.subscription = self.create_subscription(Pose2D, "ukf_pose", self.ukf_callback, 10)
        
    def declare_pd_parameters(self):
        # Declare parameters for target
        self.declare_parameter("target_x", 10.0)
        self.declare_parameter("target_y", 10.0)

        # Declare parameters for velocity limits
        self.declare_parameter("v_max", 0.25)
        self.declare_parameter("omega_max", 0.1)
        
        # Declare parameters for PD gains
        self.declare_parameter("kp_linear", 1.0)
        self.declare_parameter("kd_linear", 0.1)
        self.declare_parameter("kp_angular", 2.0)
        self.declare_parameter("kd_angular", 0.1)
    
    def ukf_callback(self, msg: Pose2D):
        # Get the time difference dt
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds * 1e-9
        if dt <= 0.0:
            dt = 0.1 

        # Compute the position error
        error_x = self.target_x - msg.x
        error_y = self.target_y - msg.y
        error_distance = math.sqrt(error_x**2 + error_y**2)
        
        # Compute the desired heading toward target
        desired_theta = math.atan2(error_y, error_x)
        error_theta = self.normalize_angle(desired_theta - msg.theta)
        
        # Compute PD terms (derivatives)
        derivative_linear = (error_distance - self.prev_error_linear) / dt
        derivative_angular = (error_theta - self.prev_error_angular) / dt
        
        # Compute control commands
        v = self.kp_linear * error_distance + self.kd_linear * derivative_linear
        w = self.kp_angular * error_theta + self.kd_angular * derivative_angular

        # Limit velocities
        v = min(max(-self.v_max, v), self.v_max)
        w = min(max(-self.omega_max, w), self.omega_max)
        
        # Update previous error and time
        self.prev_error_linear = error_distance
        self.prev_error_angular = error_theta
        self.prev_time = current_time
        
        # Publish the Twist command
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)
        
        self.get_logger().info(f"Published cmd: v={v:.2f}, w={w:.2f}")
        
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    pd_controller = PDController()
    rclpy.spin(pd_controller)
    pd_controller.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
