from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get path to config file
    config_dir = get_package_share_directory('autonomous_explorer')
    config_file = os.path.join(config_dir, 'config', 'explorer_params.yaml')
    
    return LaunchDescription([
        # Just the explorer node - launch Gazebo separately
        Node(
            package='autonomous_explorer',
            executable='explorer_node',
            name='explorer',
            output='screen',
            parameters=[
                config_file,
                {'use_sim_time': True}
            ]
        )
    ])