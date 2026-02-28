# Face_detection_opencv_ROS2

# ROS 2 Real-Time Face Detection Ecosystem

A modular ROS 2 (Humble) package that captures a live video feed, broadcasts the frames across the ROS network, and performs real-time face detection using OpenCV Haar Cascades. This package serves as a foundational vision module for human-robot interaction and social robotics applications.



## 🚀 System Architecture

This ecosystem is separated into two distinct ROS 2 Python nodes to maintain modularity:

1. **`video_publisher`**: Interfaces with the system's webcam (or a video file), captures frames, converts them using `cv_bridge`, and publishes them as `sensor_msgs/Image` to the `/video_frames` topic.
2. **`face_detector`**: Subscribes to the `/video_frames` topic, converts the ROS messages back to OpenCV format, processes the grayscale image through a pre-trained Haar Cascade classifier, and publishes the live count of detected faces as `std_msgs/Int32` to the `/face_count` topic.

## 🛠️ Prerequisites

* **OS:** Ubuntu 22.04
* **ROS Version:** ROS 2 Humble
* **Dependencies:** Python 3, OpenCV

To install the required ROS system dependencies, run:
```bash
sudo apt update
sudo apt install ros-humble-cv-bridge ros-humble-sensor-msgs ros-humble-std-msgs
```

