from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rviz_config=get_package_share_directory('particle_life')+'/rviz/rviz.rviz'
    return LaunchDescription([
        Node(
            package='particle_life',
            executable='particle_life_node',
            name='particle_life',
            output='screen',
            parameters=[{
                'num_particles': 600,
                'update_rate': 10,           # ms
                'world_radius': 10.0,
                'neighbor_radius': 1.0,# boidsとみなす近傍距離
                'max_speed': 0.05,
                'separation_weight': 0.001,# boidsの分離の強さ
                'alignment_weight': 0.05,# boidsの整列の強さ
                'cohesion_weight': 0.05,# boidsの凝集の強さ
            }]
        ),
        Node(namespace='rviz2', package='rviz2', executable='rviz2', arguments=['-d',rviz_config])
    ])