#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Twist
import math

class Robot(Node):
    def __init__(self):
        super().__init__("robot_node")
        
        # Initialize state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.dt = 0.1  
        
        # Create a publisher for the robot's pose
        self.pose_pub = self.create_publisher(Pose2D, "robot_pose", 10)
        
        # Create a subscriber for velocity commands from the PD controller
        self.cmd_sub = self.create_subscription(Twist, "cmd_vel", self.cmd_callback, 10)
        
        # Initialize commands
        self.current_v = 0.0
        self.current_w = 0.0
        
        # Timer to update the robot's state
        self.timer = self.create_timer(self.dt, self.update_state)
        
    def cmd_callback(self, msg: Twist):
        # Update the latest control commands
        self.current_v = msg.linear.x
        self.current_w = msg.angular.z
        self.get_logger().info(f"Received cmd: v={self.current_v:.2f}, w={self.current_w:.2f}")
        
    def update_state(self):
        # Unicycle dynamics: update state based on v and w
        self.x += self.current_v * math.cos(self.theta) * self.dt
        self.y += self.current_v * math.sin(self.theta) * self.dt
        self.theta += self.current_w * self.dt
        self.theta = self.normalize_angle(self.theta)
        
        # Publish the updated state
        pose_msg = Pose2D()
        pose_msg.x = self.x
        pose_msg.y = self.y
        pose_msg.theta = self.theta
        self.pose_pub.publish(pose_msg)
        self.get_logger().info(f"Published robot state: x={self.x:.2f}, y={self.y:.2f}, theta={self.theta:.2f}")
        
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    robot = Robot()
    rclpy.spin(robot)
    robot.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
