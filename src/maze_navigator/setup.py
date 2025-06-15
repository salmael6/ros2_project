from setuptools import find_packages, setup

package_name = 'maze_navigator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='salma',
    maintainer_email='salma@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'occupancy_grid_node = maze_navigator.occupancy_grid_node:main',
            'astar_planner_node = maze_navigator.astar_planner_node:main',
            'path_follower_node = maze_navigator.path_follower_node:main',
        ],
    },
)
