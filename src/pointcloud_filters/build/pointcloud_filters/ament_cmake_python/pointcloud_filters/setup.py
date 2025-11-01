from setuptools import find_packages
from setuptools import setup

setup(
    name='pointcloud_filters',
    version='0.1.0',
    packages=find_packages(
        include=('pointcloud_filters', 'pointcloud_filters.*')),
)
