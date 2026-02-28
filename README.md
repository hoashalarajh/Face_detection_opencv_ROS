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

---

# 📦 Installation & Build
1. Create a ROS 2 workspace (if you don't have one):
``` bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. Clone this repository into your `src` directory:
```bash
git clone [https://github.com/yourusername/face_detection_pkg.git](https://github.com/yourusername/face_detection_pkg.git)
```

3. Build package:
```bash
cd ~/ros2_ws
colcon build --packages-select face_detection_pkg
```

4. Source the workspace:
```bash
source install/setup.bash
```
# 💻 Usage

The easiest way to run the ecosystem is using the provided ROS 2 launch file. This will spin up both the video publisher and the face detector simultaneously.

```bash
ros2 launch face_detection_pkg face_detection.launch.py
```

(Optional) Open a separate sourced terminal to monitor the raw face count data being published in real-time:

```bash
ros2 topic echo /face_count
```

---

## Another Method (Without Launch File):

To run the nodes individually for debugging, you can start them in separate terminals:


### Terminal 1:

```bash
source install/setup.bash
ros2 run face_detection_pkg video_publisher
```

### Terminal 2:

```bash
source install/setup.bash
ros2 run face_detection_pkg face_detector
```

---

(Optional) Open a separate sourced terminal to monitor the raw face count data being published in real-time:

```bash
source install/setup.bash
ros2 topic echo /face_count
```

---

# ⚙️ Configuration Notes
* Here we read the video from a video file. To do so, please change the line `video_path = os.path.expanduser('~/Downloads/walking.mp4')` in the file `video_publisher.py` to the video file that you want to experiment with.

* Haar Cascade Path: The `face_detector` node points to the absolute Ubuntu system path for the OpenCV Haar Cascades `(/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml)`.

  ---
