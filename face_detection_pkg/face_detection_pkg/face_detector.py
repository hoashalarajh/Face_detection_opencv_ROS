import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2

class FaceDetector(Node):
    def __init__(self):
        super().__init__('face_detector')
        # subscribe to the video frames
        self.subscription = self.create_subscription(
            Image,
            'video_frames',
            self.listener_callback,
            10
        )

        # publisher for the face count
        self.count_publisher = self.create_publisher(
            Int32,
            'face_count',
            10
        )
        self.bridge = CvBridge()

        # load the famous opencv Haar Cascade for frontal faces
        self.face_cascade = cv2.CascadeClassifier('/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml')

    def listener_callback(self, data):
        # convert ROS msg to opencv format
        frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=4)

        # publish the count
        count_msg = Int32()
        count_msg.data = len(faces)
        self.count_publisher.publish(count_msg)

        self.get_logger().info(f"Faces Detected: {len(faces)}")

        # display bounding boxes
        for (x, y, w, h) in faces:
            cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
        cv2.imshow("Face Detection (Press 'q' to close)", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = FaceDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
