import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from a_star import AStarPlanner
from rrt_star import RRTStarPlanner
from rl_agent import RLAgent

class RL_HA_RRT_Planner(Node):
    def __init__(self):
        super().__init__('rl_ha_rrt_planner')
        self.path_pub = self.create_publisher(Path, 'planned_path' , 10)
        self.goal_sub = self.create_subscriber(PoseStamped, 'goal_pose', self.goal_callback, 10)
        self.a_star = AStarPlanner()
        self.rrt_star = RRTStarPlanner()
        self.rl_agent = RLAgent()
    
    def goal_callback(self, msg):
        goal = (msg.pose.position.x, msg.pose.position.y)
        path = self.compute_path((0, 0), goal)
        self.publish_path(path)

    def compute_path(self, start, goal):
        global_path = self.a_star.plan(start, goal)
        refined_path = self.rrt_star.refined_path(global_path)
        optimized_path = self.rl_agent.optimize_path(refined_path)
        return optimized_path
    
    def publish_path(self, path):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        for x, y in path:
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)
rclpy.init()
planner = RL_HA_RRT_Planner()
rclpy.spin(planner)
rclpy.shutdown()