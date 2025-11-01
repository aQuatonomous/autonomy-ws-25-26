from setuptools import setup

package_name = 'my_lidar_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=[
        'setuptools',
        'numpy',
    ],
    zip_safe=True,
    maintainer='Jacob',
    maintainer_email='20jasb1@queensu.ca',
    description='ROS 2 package for Unitree LiDAR processing',
    license='Apache-2.0',
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/my_lidar_pkg']),
        ('share/my_lidar_pkg', ['package.xml']),
    ],
    entry_points={
        'console_scripts': [
            'lidar_subscriber = my_lidar_pkg.lidar_subscriber:main',
        ],
    },
)

