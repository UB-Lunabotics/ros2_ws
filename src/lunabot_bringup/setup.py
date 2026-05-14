import os
from glob import glob
from setuptools import setup

package_name = 'lunabot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ashish',
    maintainer_email='ashish@todo.todo',
    description='UB Lunabotics rover autonomy nodes',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ir_sensor_publisher = rover_autonomy.ir_sensor_publisher:main',
            'jetson_serial_sender = rover_autonomy.python_sender_node:main',
            'brain_node = rover_autonomy.brain_node:main',
            'mock_rover = rover_autonomy.mock_rover:main',
            'pose_initializer = rover_autonomy.pose_initializer:main',
        ],
    },
)