# SuperPoint & SuperGlue Perception Node
import rclpy
from rclpy.node import Node
import torch
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from models.superpoint import SuperPoint
from models.superglue import SuperGlue

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        self.bridge = CvBridge()

        # Subscribe to Camera
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        # Publisher for Matched Keypoints
        self.matches_pub = self.create_publisher(Image, '/keypoint_matches', 10)

        # Load SuperPoint & SuperGlue Models
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.superpoint = SuperPoint({'nms_radius': 4, 'keypoint_threshold': 0.005, 'max_keypoints': 1024}).to(self.device).eval()
        self.superglue = SuperGlue({'weights': 'outdoor'}).to(self.device).eval()

        self.prev_keypoints, self.prev_descriptors, self.prev_frame = None, None, None

    def extract_features(self, image):
        """Extract keypoints using SuperPoint"""
        img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        img_tensor = torch.from_numpy(img_gray).float()[None, None].to(self.device) / 255.0

        with torch.no_grad():
            output = self.superpoint({'image': img_tensor})

        keypoints = output['keypoints'][0].cpu().numpy()
        descriptors = output['descriptors'][0].cpu().numpy()
        return keypoints, descriptors

    def match_features(self, keypoints1, descriptors1, keypoints2, descriptors2):
        """Match features using SuperGlue"""
        keypoints1_tensor = torch.tensor(keypoints1, dtype=torch.float32).unsqueeze(0).to(self.device)
        keypoints2_tensor = torch.tensor(keypoints2, dtype=torch.float32).unsqueeze(0).to(self.device)
        descriptors1_tensor = torch.tensor(descriptors1, dtype=torch.float32).unsqueeze(0).to(self.device)
        descriptors2_tensor = torch.tensor(descriptors2, dtype=torch.float32).unsqueeze(0).to(self.device)

        input_data = {
            'keypoints0': keypoints1_tensor,
            'keypoints1': keypoints2_tensor,
            'descriptors0': descriptors1_tensor,
            'descriptors1': descriptors2_tensor,
        }

        with torch.no_grad():
            output = self.superglue(input_data)

        matches = output['matches0'][0].cpu().numpy()
        return matches

    def image_callback(self, msg):
        """ROS2 Callback for Camera Images"""
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        keypoints, descriptors = self.extract_features(frame)

        if self.prev_frame is not None:
            matches = self.match_features(self.prev_keypoints, self.prev_descriptors, keypoints, descriptors)

            # Publish Matched Keypoints
            matched_frame = cv2.drawMatches(
                self.prev_frame, [cv2.KeyPoint(x, y, 1) for x, y in self.prev_keypoints],
                frame, [cv2.KeyPoint(x, y, 1) for x, y in keypoints],
                [cv2.DMatch(i, i, 0) for i in range(len(matches)) if matches[i] > -1], None)
            
            self.matches_pub.publish(self.bridge.cv2_to_imgmsg(matched_frame, "bgr8"))

        self.prev_keypoints, self.prev_descriptors, self.prev_frame = keypoints, descriptors, frame

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
