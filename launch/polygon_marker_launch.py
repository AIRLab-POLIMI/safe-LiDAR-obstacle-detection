from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='challenge2',
            executable='polygon_marker_node.py',
            name='polygon_marker_node',
            output='screen',
            parameters=[
                {#'csv_file_path': get_package_share_directory("challenge2") + '/config/known_obs_coord.csv',
                 'csv_file_path': '/home/airlab/fira_custom_workspace/src/fira_challenge/challenge2/config/known_obs_coord_test_stage2_4dv.csv',
                 'markers_frame_id': 'map'}
            ],
        ),
    ])
