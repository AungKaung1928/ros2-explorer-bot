from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    pkg_dir = os.path.dirname(__file__)
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, 'gazebo_world.launch.py'))
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, 'slam_launch.py'))
        ),
        
        Node(
            package='autonomous_explorer',
            executable='explorer_node',
            name='explorer',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])