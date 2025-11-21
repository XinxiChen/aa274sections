import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = get_package_share_directory('your_package_name')
    
    # RViz configuration
    rviz_config = os.path.join(pkg_share, 'rviz', 'explorer.rviz')
    
    return LaunchDescription([
        # Navigator node (from HW2)
        Node(
            package='your_package_name',
            executable='navigator',
            name='navigator',
            output='screen'
        ),
        
        # Frontier explorer node
        Node(
            package='your_package_name',
            executable='frontier_explorer',
            name='frontier_explorer',
            output='screen'
        ),
        
        # RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),
    ])