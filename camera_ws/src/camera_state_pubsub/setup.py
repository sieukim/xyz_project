from setuptools import find_packages, setup

package_name = 'camera_state_pubsub'

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
    maintainer='deepet',
    maintainer_email='shimsungwhan@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'camera_state_publisher = camera_state_pubsub.camera_state_publisher:main',
        'camera_state_subscriber = camera_state_pubsub.camera_state_subscriber:main',
        'yolo_viewer = camera_state_pubsub.yolo_viewer:main',            
        ],
    },
)
