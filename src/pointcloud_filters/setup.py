from setuptools import setup

package_name = 'pointcloud_filters'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/lidar_range_filter.launch.py']),
        ('share/' + package_name, ['README.md']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='jacob',
    maintainer_email='jacob@example.com',
    description='Point cloud filtering utilities for LiDAR: range and z clipping.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_range_filter = pointcloud_filters.lidar_range_filter:main',
        ],
    },
)
