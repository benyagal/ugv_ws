from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ugv_gripper'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dudu',
    maintainer_email='dudu@todo.todo',
    description='Gripper arm control (direct Jetson GPIO servos + relay actuator + homing) running on the Jetson',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gripper_node = ugv_gripper.gripper_node:main',
            'gripper_arduino_node = ugv_gripper.gripper_arduino_node:main',
            'gripper_joy_ctrl = ugv_gripper.gripper_joy_ctrl:main',
        ],
    },
)
