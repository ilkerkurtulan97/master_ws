import os
from glob import glob
from setuptools import setup

package_name = 'zed_speed_governor'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ilker Kurtulan',
    maintainer_email='ikurtulan97@gmail.com',
    description='ZED 2i human-aware speed governor for the rover.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'zed_speed_governor_node = zed_speed_governor.governor_node:main',
        ],
    },
)
