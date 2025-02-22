import rclpy
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Float32

class BuoyancyController(Node):
    def __init__(self):
        super().__init__('buoyancy_controller')
        self.subscriber_pressure = self.create_subscription(FluidPressure, '/depth_sensor', self.depth_callback, 10)
        self.ballast_pub = self.create_publisher(Float32, '/ballast_control', 10)
        self.target_depth = 5.0  # Example: Maintain at 5m depth

    def depth_callback(self, msg):
        """Adjust buoyancy dynamically"""
        current_depth = self.calculate_depth_from_pressure(msg.fluid_pressure)

        # Adjust ballast tanks if depth error is large
        error = self.target_depth - current_depth
        ballast_adjustment = 0.1 * error  # Simple proportional control
        self.ballast_pub.publish(Float32(data=ballast_adjustment))
        self.get_logger().info(f"Depth: {current_depth:.2f}m | Ballast Adj: {ballast_adjustment:.2f}")

    def calculate_depth_from_pressure(self, pressure):
        """Convert fluid pressure to depth (meters)"""
        rho = 1025  # Density of seawater (kg/m^3)
        g = 9.81  # Gravity (m/s^2)
        depth = pressure / (rho * g)  # Depth in meters
        return depth

rclpy.init()
node = BuoyancyController()
rclpy.spin(node)
rclpy.shutdown()
