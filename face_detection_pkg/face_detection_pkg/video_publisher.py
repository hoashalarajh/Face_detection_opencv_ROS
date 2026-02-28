import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        self.publisher_ = self.create_publisher(Image, 'video_frames', 10)
        # publsih at 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Replace the path with your actual file path
        # video_path = os.path.expanduser('~/Downloads/walking.mp4')
        # now we could direct the video feed from our WebCam
        self.cap = cv2.VideoCapture(0) # replace 0, with "video_path" to feed the video from any video file
        self.bridge = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding = "bgr8")
            self.publisher_.publish(msg)


def main(args = None):
    rclpy.init(args=args)
    node = VideoPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


