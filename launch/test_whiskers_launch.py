from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description()-> LaunchDescription:

    lidar_node = Node(
        package='braitenbug',
        executable='fake_lidar',
        parameters=[{
            'sigma': 30.0
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments = f'-d {(get_package_share_directory('braitenbug'))}/config/test_false_lidar.rviz'
    )

    whiskers_node = Node(
        package='braitenbug',
        executable='whiskers-node',
        parameters=[{
            'dist_min': 0.2,
            'dist_max': 2.0
        }]
    )

    description = LaunchDescription()
    description.add_action(lidar_node)
    description.add_action(rviz_node)
    description.add_action(whiskers_node)
    return description