import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'towerlight_maker'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),         
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cg',
    maintainer_email='natthawejumjai@gmail.com',
    description='control ESP32+DFPlayer via UDP JSON',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'tower_udp_client = towerlight_maker.tower_udp_client:main',
            'tower_rules_bridge = towerlight_maker.tower_rules_bridge:main',            
        ],
    },
)
