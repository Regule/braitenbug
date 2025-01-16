from launch import LaunchDescription
from launch.events import Shutdown
from launch.actions import EmitEvent
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

    #rviz_config_path = f'{(get_package_share_directory('braitenbug'))}/config/test_false_lidar.rviz'
    #print(f'RVIZ_CONFIG_PATH = {rviz_config_path}')
    #rviz_node = Node(
    #    package='rviz2',
    #    executable='rviz2',
    #    arguments = ['-d', rviz_config_path]
    #)

    visualization_node = Node(
        package='braitenbug',
        executable='whiskers_experiment.py',
        on_exit = [EmitEvent(event=Shutdown(reason='Closing launch because UI was closed.'))]
    )

    description = LaunchDescription()
    description.add_action(lidar_node)
    #description.add_action(rviz_node)
    description.add_action(visualization_node)
    return description