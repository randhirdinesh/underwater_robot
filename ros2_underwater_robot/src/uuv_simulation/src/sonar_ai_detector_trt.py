import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit

class SonarAIDetectorTRT(Node):
    def __init__(self):
        super().__init__('sonar_ai_detector_trt')
        self.subscriber = self.create_subscription(PointCloud2, '/sonar/points', self.sonar_callback, 10)
        
        # Load TensorRT model
        self.trt_logger = trt.Logger(trt.Logger.WARNING)
        with open("/home/user/ros2_ws/models/sonar_ai_model.trt", "rb") as f:
            runtime = trt.Runtime(self.trt_logger)
            self.engine = runtime.deserialize_cuda_engine(f.read())

        self.context = self.engine.create_execution_context()
        self.input_size = (1, 64)  # Input shape
        self.input_data = cuda.mem_alloc(64 * 4)  # Allocate GPU memory

    def sonar_callback(self, msg):
        """Process sonar point cloud & classify obstacles using TensorRT"""
        pointcloud = np.random.rand(64).astype(np.float32)  # Placeholder for actual sonar data

        # Copy data to GPU
        cuda.memcpy_htod(self.input_data, pointcloud)
        
        # Run AI Model in TensorRT
        output = np.zeros(3, dtype=np.float32)
        output_mem = cuda.mem_alloc(output.nbytes)
        self.context.execute_v2([int(self.input_data), int(output_mem)])
        cuda.memcpy_dtoh(output, output_mem)

        prediction = np.argmax(output)
        obstacle_type = ["Rock", "Open Space", "Moving Object"][prediction]

        self.get_logger().info(f"TensorRT Detected: {obstacle_type}")

rclpy.init()
node = SonarAIDetectorTRT()
rclpy.spin(node)
rclpy.shutdown()
