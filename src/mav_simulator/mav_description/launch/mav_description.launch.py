#!/usr/bin/env python3

"""
MAV Robot Description Launch File (ROS 2)
Publishes the robot description and starts RViz. Supports Crazyflie and X3 drone models.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context):
    from ament_index_python.packages import get_package_share_directory

    pkg_share = get_package_share_directory('mav_description')
    model = context.perform_substitution(LaunchConfiguration('model'))

    if model == 'x3':
        xacro_file = os.path.join(pkg_share, 'urdf', 'x3_description.urdf.xacro')
    else:
        xacro_file = os.path.join(pkg_share, 'urdf', 'crazyflie_description.urdf.xacro')

    robot_description_content = Command([
        'xacro ', TextSubstitution(text=xacro_file),
        ' name:=', LaunchConfiguration('name'),
    ])
    robot_description_param = ParameterValue(robot_description_content, value_type=str)

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_rviz = LaunchConfiguration('start_rviz')

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{
            'robot_description': robot_description_param,
            'use_sim_time': use_sim_time
        }],
        condition=IfCondition(start_rviz)
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{
            'robot_description': robot_description_param,
            'publish_frequency': 30.0,
            'use_sim_time': use_sim_time
        }]
    )

    default_rviz_config = PathJoinSubstitution([FindPackageShare('mav_description'), 'rviz', 'mav.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(start_rviz)
    )

    return [joint_state_publisher, robot_state_publisher, rviz_node]


def generate_launch_description():
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='x3',
        description='Robot model: x3 or crazyflie'
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='/',
        description='Namespace for the robot'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time (set true when running with sim)'
    )

    start_rviz_arg = DeclareLaunchArgument(
        'start_rviz',
        default_value='true',
        description='Start RViz to visualize the robot'
    )

    robot_name_arg = DeclareLaunchArgument(
        'name',
        default_value='X3',
        description='Robot name passed to xacro (e.g. name:=X3 or name:=mav1)'
    )

    return LaunchDescription([
        model_arg,
        namespace_arg,
        use_sim_time_arg,
        start_rviz_arg,
        robot_name_arg,
        OpaqueFunction(function=launch_setup),
    ])

