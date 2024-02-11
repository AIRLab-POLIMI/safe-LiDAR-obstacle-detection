from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='challenge2',
            executable='lidar_filtering_node',
            name='lidar_filtering_node',
            output='screen',
            parameters=[
            {'use_sim_time': True,
            'lidar_topic': '/robot/lidar/points',
            "imu_topic": "/robot/imu/data",
            'cube_edge_length': 14.0,
            'z_lower_limit': 0.55 - 0.775,
            'x_lower_limit': -14  / 2.0 - 1.0,
            'x_upper_limit': 14 / 2.0 - 1.0,
            'perform_downsampling': False,
            'csv_file_path': get_package_share_directory("challenge2") + '/config/known_obs_coord_test_stage2_4dv.csv',
            'obstacle_coordinates_frame_id': 'map',
            'perform_azimut_angle_filtering': True,
            'minimal_azimut_angle': -110.0, # angle in deg 
            'maximal_azimut_angle': 110.0}, # angle in deg
            ],
        ),
    ])

