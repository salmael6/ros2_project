from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'self_balancing_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Install world files (if needed)
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.pgm')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.yaml')),
        # Install urdf (xacro) files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.sdf')),
        
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='salma',
    maintainer_email='salma@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['balance_controller = self_balancing_bot.balance_controller:main',
        'dynamic_tf_publisher = self_balancing_bot.dynamic_tf_publisher:main',
        'static_map_publisher = self_balancing_bot.static_map_publisher:main',
        'path_publisher = self_balancing_bot.path_publisher:main',
        'map_publisher = self_balancing_bot.map_publisher:main',
        ],
    },
)
