from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description()-> LaunchDescription:

    MIN_DISTANCE: float = 0.2
    MAX_DISTANCE: float = 3.0

    rviz_config_path = f'{(get_package_share_directory('braitenbug'))}/config/test_false_lidar.rviz'
    print(f'RVIZ_CONFIG_PATH = {rviz_config_path}')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        parameters=[
                {'log_level': ParameterValue('DEBUG', value_type=str)},
        ],
        arguments = ['-d', rviz_config_path]
    )


    rqt = ExecuteProcess(
            cmd=[
                'rqt',
                '--perspective-file',
                os.path.join(
                        get_package_share_directory('braitenbug'),
                        'config',
                        'whiskers_test.perspective'
                )
            ],
            output='screen'
        )
    
    lidar_node = Node(
        package='braitenbug',
        executable='fake_lidar',
        parameters=[{
            'sigma': 30.0,
            'min_range': MIN_DISTANCE,
            'max_range': MAX_DISTANCE
        }]
    )

    

    whiskers_node = Node(
        package='braitenbug',
        executable='whiskers-node',
        parameters=[{
            'dist_min': MIN_DISTANCE,
            'dist_max': MAX_DISTANCE
        }]
    )

    visualization_node = Node(
        package='braitenbug',
        executable='visualize_whiskers.py'
    )

    description = LaunchDescription()
    description.add_action(rqt)
    description.add_action(lidar_node)
    description.add_action(rviz_node)
    description.add_action(whiskers_node)
    description.add_action(visualization_node)
    return description