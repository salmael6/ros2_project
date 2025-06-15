from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('self_balancing_bot')
    robot_file = os.path.join(pkg_share, 'urdf', 'self_balancing_bot.sdf')
    world_file = os.path.join(pkg_share, 'worlds', 'world.sdf')

    return LaunchDescription([
        # 1. Launch Gazebo with your world
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_file],
            output='screen'
        ),

        # 2. Spawn the SDF robot directly
        TimerAction(
            period=5.0,  # wait 5 seconds before spawning robot
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'run', 'ros_gz_sim', 'create',
                        '-name', 'self_balancing_bot',
                        '-x', '0', '-y', '0', '-z', '0.3',
                        '-file', robot_file
                    ],
                    output='screen'
                )
            ]
        ),

        # robot_state_publisher node 
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_description': open(robot_file).read()
            }]
        ),


        # 3. ROS-Gazebo Bridge for common topics
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
'/tf_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'

            ],
            output='screen'
        ),
        
        # 4. Self-balancing controller
        Node(
           package='self_balancing_bot',
           executable='balance_controller',
           name='balance_controller',
           output='screen',
           parameters=[{'use_sim_time': True}]
        ),
        
        #Rviz 
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'self_balancing_bot.rviz')],
            parameters=[{'use_sim_time': True}]
        ),
        
        # 5. keyboard teleop
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop',
            output='screen',
            prefix='gnome-terminal --',
            remappings=[
                ('/cmd_vel', '/teleop_cmd_vel')
            ]
        ),
        
        Node(
           package='self_balancing_bot',
           executable='path_publisher',
           name='path_publisher',
           output='screen',
           parameters=[{'use_sim_time': True}]
        ),
        
        Node(
            package='maze_navigator',
            executable='occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        

        Node(
            package='maze_navigator',
            executable='path_follower_node',
            name='path_follower_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_broadcaster',
            arguments=['--x', '-4',
                '--y', '-4',
                '--z', '0',
                '--roll', '0',
                '--pitch', '0',
                '--yaw', '0',
                '--frame-id', 'map',
                '--child-frame-id', 'odom'
            ],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_chassis_broadcaster',
            arguments=['-4', '-4', '0', '0', '0', '0', 'odom', 'chassis'],
            output='screen'
        )




    ])

