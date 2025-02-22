#  **AI-Based ROS2 Underwater Navigation System**, 

✅ **Gazebo Simulation Setup**  
✅ **AI-Based Obstacle Avoidance with TensorRT**  
✅ **MPPI-Based AI Navigation**  
✅ **Thruster Control & Adaptive Depth Management**  
✅ **Swarm Communication & Multi-Robot AI**  
✅ **GitHub CI/CD for Automated ROS2 Build & Test**  
✅ **Docker Containerization for Cross-Platform Deployment**  

---

# 📌 **2️⃣ Setup & Installation**
## **🔹 Install ROS2 Humble & Dependencies**
```bash
sudo apt update && sudo apt install -y \
ros-humble-gazebo-ros-pkgs \
ros-humble-nav2-bringup \
python3-colcon-common-extensions \
ros-humble-ament-cmake \
ros-humble-rclcpp \
ros-humble-sensor-msgs \
ros-humble-nav-msgs \
ros-humble-geometry-msgs \
ros-humble-std-msgs \
ros-humble-tf2-ros
```

## **🔹 Build & Source the ROS2 Workspace**
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

# 📌 **3️⃣ Running Gazebo Simulation**
```bash
ros2 launch uuv_simulation uuv_gazebo.launch.py
```
✅ **Spawns the underwater robot in a realistic ocean environment.**  
✅ **Simulates thruster dynamics, buoyancy, and sonar/lidar interactions.**  

---

# 📌 **4️⃣ Running AI-Based Obstacle Avoidance**
## **🔹 Convert AI Models to TensorRT for Faster Processing**
```bash
python3 convert_sonar_model.py
python3 convert_avoidance_model.py
```

## **🔹 Start AI-Based Sonar Object Detector**
```bash
ros2 run uuv_simulation sonar_ai_detector_trt.py
```

## **🔹 Start AI-Based Path Optimization**
```bash
ros2 run uuv_simulation ai_obstacle_avoidance.py
```
✅ **Uses AI-powered deep learning models to classify sonar obstacles in real-time.**  
✅ **TensorRT optimizations enable ultra-fast inference on Jetson NX.**  

---

# 📌 **5️⃣ Running MPPI-Based AI Navigation**
```bash
ros2 launch rl_ha_rrt navigation.launch.py
```
✅ **MPPI dynamically optimizes paths for smooth motion.**  
✅ **GPU-accelerated path planning ensures real-time adjustments.**  

---

# 📌 **6️⃣ Running Thruster Control & Adaptive Depth Management**
```bash
ros2 run uuv_simulation thruster_controller.py
ros2 launch robot_localization ekf.launch.py
```
✅ **PID-controlled thrusters manage depth & position stability.**  
✅ **Thruster forces are optimized based on AI predictions.**  

---

# 📌 **7️⃣ Running Multi-Robot Swarm Communication**
```bash
ros2 run uuv_simulation swarm_manager.py
```
✅ **Each robot communicates position, path updates, and mission status.**  

---

# 📌 **8️⃣ Debugging & Fine-Tuning**
## **🔹 Check if All ROS Topics Are Publishing**
```bash
ros2 topic list
```
Ensure you see:
```
/odometry/filtered
/lidar/points
/sonar/points
/zedx/image_raw
/planned_path
/ai_optimized_path
/tf
```

## **🔹 Debugging Common Issues**
| **Issue** | **Solution** |
|-----------|-------------|
| **No data in `/lidar/points` or `/sonar/points`** | Check hardware connections & restart sensors. |
| **Robot is drifting too much in depth** | Adjust `PID` gains in `thruster_controller.py`. |
| **Gazebo is running too slowly** | Reduce `real_time_update_rate` in `underwater.world`. |
| **MPPI not computing paths fast enough** | Increase `batch_size` in `controller_mppi.yaml` and enable GPU. |
| **AI is detecting obstacles incorrectly** | Retrain AI model with better sonar data. |

---



# 📌 **🔟 Docker Containerization for Cross-Platform Deployment**
📄 **File: `Dockerfile`**
```dockerfile
FROM osrf/ros:humble-desktop

# Install dependencies
RUN apt update && apt install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-nav2-bringup \
    python3-colcon-common-extensions \
    ros-humble-ament-cmake \
    ros-humble-rclcpp \
    ros-humble-sensor-msgs \
    ros-humble-nav-msgs \
    ros-humble-geometry-msgs \
    ros-humble-std-msgs \
    ros-humble-tf2-ros \
    && rm -rf /var/lib/apt/lists/*

# Set up ROS2 environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN mkdir -p /ros2_ws/src

# Copy workspace files
COPY ./ros2_ws /ros2_ws
WORKDIR /ros2_ws

# Build ROS2 packages
RUN colcon build
```

### **🔹 Run ROS2 in Docker**
```bash
docker build -t ros2_underwater .
docker run -it --rm --net=host ros2_underwater bash
```

---

