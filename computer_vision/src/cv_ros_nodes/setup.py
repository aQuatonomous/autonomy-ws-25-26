from setuptools import find_packages, setup

package_name = 'cv_ros_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
        ['cv_ros_nodes/launch/launch_cv.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lorenzo',
    maintainer_email='lorenzodemarni666@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vision_preprocessing = cv_ros_nodes.vision_preprocessing:main',
            'vision_inference = cv_ros_nodes.vision_inference:main',
            'vision_combiner = cv_ros_nodes.vision_combiner:main',
            'camera_viewer = cv_ros_nodes.camera_viewer:main',
            'task4_supply_processor = cv_ros_nodes.task4_supply_processor:main',
        ],
    },
)
