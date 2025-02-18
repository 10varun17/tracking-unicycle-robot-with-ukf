#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Twist
import numpy as np
import math
from ukf_project.ukf import UKF

class UKFNode(Node):
    def __init__(self):
        super().__init__("ukf_node")
        
        # Initialize variables
        dt = 0.1    
        initial_state = np.array([0.0, 0.0, 0.0])
        initial_cov = np.eye(3) * 0.1
        q_var = 1e-3
        Q = np.diag([q_var, q_var, 0.1])
        z_var = 0.1
        R = np.diag([z_var, z_var])
        
        # Parameters for computing sigma
        alpha = 1e-3
        beta = 2.0
        kappa = 0
        
        # UKF parameters
        dim_x = initial_state.shape[0]  # 3 here for x, y, theta
        dim_z = R.shape[0]              # 2 here for x, y
        
        # Create an instance of the UKF filter
        self.ukf = UKF(dim_x=dim_x, dim_z=dim_z, dt=dt, fx=self.fx, hx=self.hx,
                             Q=Q, R=R, alpha=alpha, beta=beta, kappa=kappa,
                             initial_state=initial_state, initial_covariance=initial_cov)
        
        # Variable for storing the time the last sensor readings were received
        self.last_update_time = self.get_clock().now()
        
        # Variable for latest control command
        self.last_cmd = None
        
        # Create susbscription to sensor_pose and cmd_vel
        self.sensor_sub = self.create_subscription(Pose2D, "sensor_pose", self.sensor_callback, 10)
        self.cmd_sub = self.create_subscription(Twist, "cmd_vel", self.cmd_callback, 10)
        
        # Create a publisher for estimated pose of the robot
        self.ukf_pub = self.create_publisher(Pose2D, "ukf_pose", 10)
        
    def fx(self, state, dt, u):
        """
        Using unicycle dynamics for process model.
        
        state: numpy array [x, y, theta]
        dt: time step
        u: control command (Twist message)
        """
        x, y, theta = state
        
        # Get v and w from control command
        if u is None:
            v = 0.0
            w = 0.0
        else:
            v = u.linear.x
            w = u.angular.z
            
        # Update x, y, theta using unicycle dynamics
        x += v * math.cos(theta) * dt
        y += v * math.sin(theta) * dt
        theta += w * dt
        theta = self.normalize_angle(theta)
        
        return np.array([x, y, theta])
    
    def hx(self, state):
        """
        Measurement function that returns the x and y components of the state.
        """
        return np.array([state[0], state[1]])
    
    def normalize_angle(self, angle):
        """
        Normalize angle to be in [-pi, pi].
        """
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def cmd_callback(self, msg: Twist):
        """
        Callback to receive the latest control command.
        """
        self.last_cmd = msg
        self.get_logger().info(f"Received cmd: v={msg.linear.x:.2f}, w={msg.angular.z:.2f}")
        
    def sensor_callback(self, msg: Pose2D):
        """
        Callback to receive the latest noisy measurement.
        """
        current_time = self.get_clock().now()
        dt = (current_time - self.last_update_time).nanoseconds * 1e-9 # elapsed time between consective filter updates
        if dt <= 0:
            dt = 0.1
        self.last_update_time = current_time
        
        # Predict step with the current control command 
        self.ukf.predict(dt, self.last_cmd)
        
        # Get the noisy measurement from the sensor
        z = np.array([msg.x, msg.y])
        
        # Update step with the measurement
        self.ukf.update(z)
        
        # Prepare and publish the estimated pose
        estimated_pose = Pose2D()
        estimated_pose.x = float(self.ukf.x[0])
        estimated_pose.y = float(self.ukf.x[1])
        estimated_pose.theta = float(self.ukf.x[2])
        self.ukf_pub.publish(estimated_pose)
        
        # Logging purpose
        x = estimated_pose.x
        y = estimated_pose.y
        
        self.get_logger().info(f"Published UKF estimated pose: x={x:.2f}, y={y:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = UKFNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()