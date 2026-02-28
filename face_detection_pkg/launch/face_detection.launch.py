from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # video publisher node
        Node(
            package = 'face_detection_pkg',
            executable = 'video_publisher',
            name = 'video_publisher',
            output = 'screen'
        ),

        # face detector node
        Node(
            package = 'face_detection_pkg',
            executable = 'face_detector',
            name = 'face_detector',
            output = 'screen'
        ),
    ])