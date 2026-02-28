'''
from setuptools import find_packages, setup

package_name = 'face_detection_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hoashal',
    maintainer_email='hoashal@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'video_publisher = face_detection_pkg.video_publisher:main',
            'face_detector = face_detection_pkg.face_detector:main',
        ],
    },
)
'''
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'face_detection_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # THIS IS THE NEW LINE YOU MUST ADD:
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hoashal',
    maintainer_email='your.email@example.com',
    description='Real-time face detection ecosystem using OpenCV Haar Cascades',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'video_publisher = face_detection_pkg.video_publisher:main',
            'face_detector = face_detection_pkg.face_detector:main',
        ],
    },
)