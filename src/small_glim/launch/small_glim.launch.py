from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os, glob

def generate_launch_description():
    use_foxglove = LaunchConfiguration('use_foxglove')
    foxglove_port = LaunchConfiguration('foxglove_port')
    configs = glob.glob(os.path.join(get_package_share_directory('small_glim'), 'config', 'params_*.yaml'))
    return LaunchDescription([
        DeclareLaunchArgument('use_foxglove', default_value='true'),
        DeclareLaunchArgument('foxglove_port', default_value='8765'),
        Node(
            package="small_glim",
            executable="small_glim_node",
            output="screen",
            emulate_tty=True,
            parameters=configs
        ),
        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            output="log",
            parameters=[{'port': ParameterValue(foxglove_port, value_type=int)}],
            condition=IfCondition(use_foxglove)
        )
    ])