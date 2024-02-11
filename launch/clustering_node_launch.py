from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='challenge2',
            executable='clustering_node',
            name='clustering_node',
            output='screen',
            parameters=[
            {'use_sim_time': True,
            'filtered_pointcloud_topic': '/filtered_pointcloud',
            'obstacle_coordinates_frame_id': 'map',
            'perform_outliers_removal': False,
            'min_cluster_size': 20,
            'sor_mean_K': 10,
            'obs_radius_threshold': 0.45,
            'cluster_tolerance': 0.45},
            ],
        ),
    ])

