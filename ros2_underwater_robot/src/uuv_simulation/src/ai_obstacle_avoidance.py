import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import torch

# AI-Based Obstacle Avoidance Model
class AvoidanceModel(torch.nn.Module):
    def __init__(self):
        super(AvoidanceModel, self).__init__()
        self.fc1 = torch.nn.Linear(4, 16)
        self.fc2 = torch.nn.Linear(16, 16)
        self.fc3 = torch.nn.Linear(16, 2)  # Output: Adjusted Path X, Y

    def forward(self, x):
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        return self.fc3(x)  # New X, Y Adjustments

# AI-Based Obstacle Avoidance Node
class AIObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('ai_obstacle_avoidance')
        self.subscriber = self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        self.pub = self.create_publisher(Path, '/ai_optimized_path', 10)
        self.model = AvoidanceModel()
        self.model.load_state_dict(torch.load('/home/user/ros2_ws/models/obstacle_avoidance_model.pth'))
        self.model.eval()

    def path_callback(self, msg):
        """Process planned path & modify using AI if obstacles detected"""
        modified_path = Path()
        modified_path.header = msg.header

        for pose in msg.poses:
            state_tensor = torch.tensor([pose.pose.position.x, pose.pose.position.y, 0, 0], dtype=torch.float32)
            adjusted_xy = self.model(state_tensor).detach().numpy()

            new_pose = PoseStamped()
            new_pose.pose.position.x = adjusted_xy[0]
            new_pose.pose.position.y = adjusted_xy[1]
            modified_path.poses.append(new_pose)

        self.pub.publish(modified_path)
        self.get_logger().info("AI Optimized Path Published!")

rclpy.init()
node = AIObstacleAvoidance()
rclpy.spin(node)
rclpy.shutdown()
