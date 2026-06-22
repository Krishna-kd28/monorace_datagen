from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'output_dir',
            default_value='~/pencilnet_dataset',
            description='Directory to save the dataset (images/ + annotations.json)'),
        DeclareLaunchArgument(
            'num_viewpoints',
            default_value='15000',
            description='Total viewpoints to sample across all gates'),
        DeclareLaunchArgument(
            'closest_gate_only',
            default_value='false',
            description='Keep only the closest gate per frame'),
        DeclareLaunchArgument(
            'trigger_topic',
            default_value='/camera_trigger',
            description='Gazebo topic that fires the triggered cameras'),

        Node(
            package='pencilnet_ros',
            executable='data_generator',
            name='data_generator',
            output='screen',
            parameters=[{
                'output_dir': LaunchConfiguration('output_dir'),
                'num_viewpoints': LaunchConfiguration('num_viewpoints'),
                'closest_gate_only': LaunchConfiguration('closest_gate_only'),
                'trigger_topic': LaunchConfiguration('trigger_topic'),
            }],
        ),
    ])
