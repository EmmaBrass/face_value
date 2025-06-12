from setuptools import setup
import os
from glob import glob

package_name = 'ros_3xs'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Emma Brass',
    maintainer_email='emma@example.com',
    description='ROS2 package for 3xs computer nodes',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'screen3_node = ros_3xs.screen3_node:main',
            'screen2_node = ros_3xs.screen2_node:main',
            'screen1_node = ros_3xs.screen1_node:main',
            'image_save_node = ros_3xs.image_save_node:main',
            'robot_node = ros_3xs.robot_node:main'
        ],
    },
)
