from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'task_1_wheeled_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # install launch, urdf, and world files
        (os.path.join('share', package_name, 'launch'), glob('task_1_wheeled_robot/launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('task_1_wheeled_robot/urdf/*')),
        (os.path.join('share', package_name, 'world'), glob('task_1_wheeled_robot/world/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='raihankabirratul@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'motion_node = task_1_wheeled_robot.ros_nodes.motion_node:main',
        ],
    },
)
