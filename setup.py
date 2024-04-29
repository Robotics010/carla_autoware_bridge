from glob import glob
import os

from setuptools import setup

package_name = 'carla_autoware_bridge'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name, 'carla_autoware_bridge.converter'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config',
         ['config/objects.json']),
        ('share/' + package_name + '/data/carla_tesla_model3',
         glob('data/carla_tesla_model3/*.csv')),
        (os.path.join('share', package_name),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robotics010',
    maintainer_email='kirill.mitkovskii@gmail.com',
    description='Convert and transfer messages between carla_ros_bridge and autoware universe',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'carla_autoware_bridge = carla_autoware_bridge.carla_autoware_bridge:main',
            'carla_service = carla_autoware_bridge.carla_service:main',
            'goal_sender = carla_autoware_bridge.goal_sender:main',
        ],
    },
)
