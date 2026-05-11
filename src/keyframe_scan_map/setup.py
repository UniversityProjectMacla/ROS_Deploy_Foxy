from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'keyframe_scan_map'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='macla',
    maintainer_email='macla@todo.todo',
    description='Keyframe scan map (merged PointCloud2 in map frame)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'keyframe_map_node = keyframe_scan_map.keyframe_map_node:main',
            'pose_graph_node = keyframe_scan_map.pose_graph_node:main',
        ],
    },
)
