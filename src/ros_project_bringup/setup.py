from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'ros_project_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'docs'), glob('docs/*.md')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        ('share/ros_project_bringup/rviz', ['rviz/slam.rviz']),
        # launch_ros Node() resolves executables under lib/<pkg>/; setuptools puts the real script in bin/.
        (
            os.path.join('lib', package_name),
            [
                'ros_project_bringup/wrappers/lidar_odom_ema_smooth',
                'ros_project_bringup/wrappers/ndt_ekf_time_diagnose',
                'ros_project_bringup/wrappers/pipeline_translation_debug',
            ],
        ),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='macla',
    maintainer_email='macla@todo.todo',
    description='ROS Project launcher',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'lidar_odom_ema_smooth = ros_project_bringup.lidar_odom_ema_smooth:main',
            'ndt_ekf_time_diagnose = ros_project_bringup.ndt_ekf_time_diagnose:main',
            'pipeline_translation_debug = ros_project_bringup.pipeline_translation_debug:main',
        ],
    },
)

