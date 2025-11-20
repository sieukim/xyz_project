import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'dsr_example'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'weights'),
        glob(os.path.join(package_name, 'weights', '*.pt'))),
        ('share/dsr_example/launch', ['launch/smart_refueler_bringup.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gossi',
    maintainer_email='mincheol710313@gmail.com',
    description='TODO: Package description',
    license='BSD',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'webcam_manager_ros = dsr_example.webcam_manager_ros:main',
                'webcam_streamer = dsr_example.webcam_streamer:main',
                'realsense_manager_ros = dsr_example.realsense_manager_ros:main',
                'realsense_streamer = dsr_example.realsense_streamer:main',
                'ocr_manager_ros = dsr_example.ocr_manager_ros:main',
                'ocr_streamer = dsr_example.ocr_streamer:main',

                'vision_target_node = dsr_example.vision_target_node:main',
                'motion_controller = dsr_example.motion_controller:main',
                'llm_narrator_client = dsr_example.llm_narrator_client:main',
            ],
        },
    
    )
