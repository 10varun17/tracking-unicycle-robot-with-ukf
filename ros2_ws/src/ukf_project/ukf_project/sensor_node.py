#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
import numpy as np

class Sensor(Node):
    def __init__(self):
        super().__init__("sensor_node")
        
        # Create a subscriber to the robot"s pose
        self.pose_sub = self.create_subscription(Pose2D, "robot_pose", self.pose_callback, 10)
        
        # Create a publisher for the noisy sensor measurement
        self.noisy_pose_pub = self.create_publisher(Pose2D, "sensor_pose", 10)
        
        # Std deviations for Gaussian Noise
        self.noise_std_x = 0.1  
        self.noise_std_y = 0.1  
        
    def pose_callback(self, msg: Pose2D):
        noisy_msg = Pose2D()
        noisy_msg.x = msg.x + np.random.normal(0.0, self.noise_std_x)
        noisy_msg.y = msg.y + np.random.normal(0.0, self.noise_std_y)
        noisy_msg.theta = msg.theta
        self.noisy_pose_pub.publish(noisy_msg)
        self.get_logger().info(f"Published noisy pose: x={noisy_msg.x:.2f}, y={noisy_msg.y:.2f}")

def main(args=None):
    rclpy.init(args=args)
    sensor = Sensor()
    rclpy.spin(sensor)
    sensor.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
