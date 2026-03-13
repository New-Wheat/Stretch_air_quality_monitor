import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'stretch_wacc_sensor'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
		(os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jingzhou Zhu',
    maintainer_email='newwheatzjz@outlook.com',
    description='Publish custom sensor data',
    license='BSD',
    entry_points={
        'console_scripts': [
            'udp_processor = stretch_wacc_sensor.udp_processor:main'
        ],
    },
)
