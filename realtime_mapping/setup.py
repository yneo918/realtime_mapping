from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'realtime_mapping'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install configuration files
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        # Install sample configuration files
        ('share/' + package_name + '/config', glob('config/*.example')),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'matplotlib',
        'PyYAML',
    ],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='ROS2 package for realtime 2D sensor mapping with heatmap visualization',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'realtime_mapper = realtime_mapping.realtime_mapper:main',
            'topic_inspector = realtime_mapping.topic_inspector:main',
            'fake_publisher = realtime_mapping.fake_publisher:main',
        ],
    },
)
