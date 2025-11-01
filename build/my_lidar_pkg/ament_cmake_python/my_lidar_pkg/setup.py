from setuptools import find_packages
from setuptools import setup

setup(
    name='my_lidar_pkg',
    version='0.1.0',
    packages=find_packages(
        include=('my_lidar_pkg', 'my_lidar_pkg.*')),
)
