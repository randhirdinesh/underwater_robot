import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import FluidPressure, Imu
import torch
import numpy as np

# Load trained AI Model (Pretrained using Reinforcement Learning)
class AIThrusterModel(torch.nn.Module):
    def __init__(self):
        super(AIThrusterModel, self).__init__()
        self.fc1 = torch.nn.Linear(4, 16)
        self.fc2 = torch.nn.Linear(16, 16)
        self.fc3 = torch.nn.Linear(16, 3)  # Output: Thrust adjustments (X, Y, Z)

    def forward(self, x):
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        return torch.tanh(self.fc3(x))  # Output values between -1 and 1

# AI-Based Thruster Controller Node
class AIThrusterOptimizer(Node):
    def __init__(self):
        super().__init__('ai_thruster_optimizer')
        self.subscriber_cmd = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.subscriber_pressure = self.create_subscription(FluidPressure, '/depth_sensor', self.depth_callback, 10)
        self.subscriber_imu = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.thrust_pub = self.create_publisher(Twist, '/ai_optimized_cmd_vel', 10)

        self.model = AIThrusterModel()
        self.model.load_state_dict(torch.load('/home/user/ros2_ws/models/thruster_rl_model.pth'))  # Load trained AI model
        self.model.eval()

        self.current_depth = 0.0
        self.current_acceleration = np.array([0.0, 0.0, 0.0])

    def cmd_vel_callback(self, msg):
        """Apply AI-based optimization to thruster force commands"""
        state = torch.tensor([self.current_depth, self.current_acceleration[0], self.current_acceleration[1], self.current_acceleration[2]], dtype=torch.float32)
        optimized_thrust = self.model(state).detach().numpy()

        # Apply optimized force to thrusters
        optimized_cmd = Twist()
        optimized_cmd.linear.x = optimized_thrust[0] * 10  # Forward/Backward thrust
        optimized_cmd.linear.y = optimized_thrust[1] * 10  # Left/Right thrust
        optimized_cmd.linear.z = optimized_thrust[2] * 10  # Depth control

        self.thrust_pub.publish(optimized_cmd)
        self.get_logger().info(f"AI Optimized Thruster Command: {optimized_cmd}")

    def depth_callback(self, msg):
        """Handle depth readings from pressure sensor"""
        self.current_depth = self.calculate_depth_from_pressure(msg.fluid_pressure)

    def imu_callback(self, msg):
        """Handle IMU data for acceleration feedback"""
        self.current_acceleration = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])

    def calculate_depth_from_pressure(self, pressure):
        """Convert fluid pressure to depth (meters)"""
        rho = 1025  # Density of seawater (kg/m^3)
        g = 9.81  # Gravity (m/s^2)
        depth = pressure / (rho * g)
        return depth

rclpy.init()
node = AIThrusterOptimizer()
rclpy.spin(node)
rclpy.shutdown()
