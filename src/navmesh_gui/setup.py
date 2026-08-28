from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'navmesh_gui'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ilker',
    maintainer_email='ikurtulan97@gmail.com',
    description='Open3D GUI for navmesh goal selection',
    license='MIT',
    entry_points={
        'console_scripts': [
            'navmesh_gui = navmesh_gui.app:main',
        ],
    },
)
