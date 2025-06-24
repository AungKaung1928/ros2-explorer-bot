from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():
    world_file = os.path.join(
        os.path.dirname(__file__), '..', 'worlds', 'maze_world.world')
    
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file],
            output='screen'
        ),
        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': open(os.path.join(
                os.path.dirname(__file__), '..', 'urdf', 'explorer_robot.urdf')).read()}]
        )
    ])