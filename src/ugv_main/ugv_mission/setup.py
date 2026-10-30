from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'ugv_mission'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'param'), glob('param/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='benji',
    maintainer_email='benji@todo.todo',
    description='Chickcheck2 mission planner',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'mission_planner = ugv_mission.mission_planner:main',
            'dock_server = ugv_mission.dock_server:main',
        ],
    },
)
