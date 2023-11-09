from os import environ
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import ExecuteProcess
from launch.conditions import LaunchConfigurationEquals, IfCondition
from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument('sync', default_value='true',
                          choices=['true', 'false'],
                          description='Run async or sync SLAM'),
    DeclareLaunchArgument('localization', default_value='slam',
                          choices=['off', 'localization', 'slam'],
                          description='Whether to run localization or SLAM'),
    DeclareLaunchArgument('nav2', default_value='true',
                          choices=['true', 'false'],
                          description='Run nav2'),
    DeclareLaunchArgument('corti', default_value='true',
                          choices=['true', 'false'],
                          description='Run corti'),
    DeclareLaunchArgument('laser', default_value='true',
                          choices=['true', 'false'],
                          description='Run laser'),
    DeclareLaunchArgument('synapse_ros', default_value='true',
                          choices=['true', 'false'],
                          description='Run synapse_ros'),
    DeclareLaunchArgument('description', default_value='true',
                          choices=['true', 'false'],
                          description='Run description'),
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('log_level', default_value='error',
                          choices=['info', 'warn', 'error'],
                          description='log level'),
]


def generate_launch_description():
    synapse_ros = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
            [get_package_share_directory('synapse_ros'), 'launch', 'synapse_ros.launch.py'])]),
        condition=IfCondition(LaunchConfiguration('synapse_ros')),
        launch_arguments=[('host', ['192.0.2.1']),
                          ('port', '4242'),
                          ('use_sim_time', LaunchConfiguration('use_sim_time'))]
    )

    laser = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
            [get_package_share_directory('b3rb_bringup'), 'launch', 'laser.launch.py'])]),
        condition=IfCondition(LaunchConfiguration('laser')),
        launch_arguments=[('stl27l', 'true'),
                          ('use_sim_time', LaunchConfiguration('use_sim_time'))]
    )

    # Robot description
    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
        [get_package_share_directory('b3rb_description'), 'launch', 'robot_description.launch.py'])]),
        condition=IfCondition(LaunchConfiguration('description')),
        launch_arguments=[('use_sim_time', LaunchConfiguration('use_sim_time'))])

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
        [get_package_share_directory(
        'b3rb_nav2'), 'launch', 'nav2.launch.py'])]),
        condition=IfCondition(LaunchConfiguration('nav2')),
        launch_arguments=[('use_sim_time', LaunchConfiguration('use_sim_time'))])

    corti = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
        [get_package_share_directory('corti'), 'launch', 'corti.launch.py'])]),
        condition=IfCondition(LaunchConfiguration('corti')),
        launch_arguments=[('use_sim_time', LaunchConfiguration('use_sim_time'))])

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
        [get_package_share_directory(
        'b3rb_nav2'), 'launch', 'slam.launch.py'])]),
        condition=LaunchConfigurationEquals('localization', 'slam'),
        launch_arguments=[('use_sim_time', LaunchConfiguration('use_sim_time')),
            ('sync', LaunchConfiguration('sync'))])

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
        [get_package_share_directory(
        'b3rb_nav2'), 'launch', 'localization.launch.py'])]),
        condition=LaunchConfigurationEquals('localization', 'localization'),
        launch_arguments=[('use_sim_time', LaunchConfiguration('use_sim_time')),
            ('map', PathJoinSubstitution([get_package_share_directory(
                'b3rb_nav2'), 'maps', LaunchConfiguration('map_yaml')]))])

    tf_to_odom = Node(
        condition=IfCondition(LaunchConfiguration('corti')),
        package='corti',
        executable='tf_to_odom',
        output='screen',
        parameters=[{
            'base_frame': 'map',
            'target_frame': 'base_link',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        remappings=[
            ('/odom', '/cerebri/in/odometry')
            ])

    odom_to_tf = Node(
        condition=IfCondition(LaunchConfiguration('corti')),
        package='corti',
        executable='odom_to_tf',
        output='screen',
        parameters=[{
            'async': False,
            'sync_dt': 0.02,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        remappings=[
            ('/odom', '/cerebri/out/odometry')
            ])

    # Define LaunchDescription variable
    return LaunchDescription(ARGUMENTS + [
        robot_description,
        synapse_ros,
        nav2,
        corti,
        slam,
        laser,
        localization,
        #tf_to_odom,
        odom_to_tf,
    ])
