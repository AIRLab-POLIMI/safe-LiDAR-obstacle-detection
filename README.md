# Safe LiDAR Obstacle Detection

This obstacle detection system has been realized for the 2024 FIRA Hackathon and tested in the competition's simulated environment. To test the package in the same environment, download and build the [fira_hackathon_workspace](https://github.com/FiraHackathon/fira_hackathon_workspace) and add the 3D LiDAR as specified below.



## Add 3D LiDAR

### 1. Disable 2D LiDAR lms151

In `fira_hackathon_workspace/src/romea_ros2/demos/fira_hackathon_demo/cfg_chal2/robots/robot/devices.yaml`

Change `available_mode` to `none`:
```
lms151:
	type: lidar
	available_mode: all -> none
```

### 2. Enable 3D LiDAR mrs1xxx on the robot

In `fira_hackathon_workspace/src/romea_ros2/demos/fira_hackathon_demo/cfg_chal2/robots/robot/devices.yaml`

Add:
```
mrs1xxx:
	type: lidar
	available_mode: all
```

### 3. Add 3D LiDAR configurations in the devices directory

In `fira_hackathon_workspace/src/romea_ros2/demos/fira_hackathon_demo/cfg_chal2/robots/robot/devices`

Add the file `mrs1xxx.lidar.yaml` with the following content:
```yaml
name: "lidar"
driver:
  pkg: "sick_scan"
  ip: "192.168.1.112"
  port: "2112"
configuration:
  type: sick
  model: mrs1000
  rate: 20
  resolution: 0.25
geometry:
  parent_link: "base_link"
  xyz: [2.02, 0.0, 0.45]
  rpy: [0.0, 0.0, 0.0]
records:
  scan: true
  cloud: false
```

### 4. Modify the sensor specifications if needed

In `fira_hackathon_workspace/src/romea_ros2/interfaces/sensors/romea_lidar/romea_lidar_description/config`

Modify the file `sick_mrs1xxx_specifications.yaml`:

```yaml
type: 3D
minimal_azimut_angle: -125.0  # in deg
maximal_azimut_angle: 125.0  # in deg
azimut_angle_increment: [0.25, 0.125, 0.0625]  # in deg
azimut_angle_std: 0.0
samples: [1001, 2001, 4001]  # number of samples = (maximal_azimut_angle-minimal_azimut_angle)/azimut_angle_increment +1
minimal_elevation_angle: -4.0  # in deg
maximal_elevation_angle: 8.09  # in deg
elevation_angle_increment: 0.39  # in deg
elevation_angle_std: 0.0
lasers: 32  # number of laser = (maximal_elevation_angle-minimal_elevation_angle)/elevation_angle_increment) +1
minimal_range: 0.5  # in meter
maximal_range: 7.0  # in meter
range_std: 0.0
rate: 20
```

### 5. 3D LiDAR topic

`/robot/lidar/points`

For better visualization in Rviz, change the style mode to "points".



## ROS package build

### 1. Install the dependency packages via the Dockerfile

(New packages are preceded by an arrow "-->")

```dockerfile
RUN --mount=type=cache,target=/var/lib/apt/lists \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        # you can add some ubuntu packages here \
        python3-pip \
        nlohmann-json3-dev \
        libgsl-dev \
        --> ros-humble-pcl-conversions \
        --> ros-humble-pcl-ros \
        --> ros-humble-rqt-tf-tree && \
```

Then go in the `fira_hackathon_workspace` directory and recompile with the command:

```bash
docker compose up compile --build
```

### 2. Connect to the Docker container with a new terminal and source the ROS workspaces

```bash
source /opt/ros/humble/setup.bash
source /home/riccardo/fira_custom_workspace/install/local_setup.bash 
```

### 3. Build the ch2_msg_srv package with Colcon

Go in the `fira_custom_workspace` directory and use the command:

```bash
colcon build --symlink-install --packages-select ch2_msg_srv
```

### 4. Build the challenge2 package with Colcon

Go in the `fira_custom_workspace` directory and use the command:

```bash
colcon build --symlink-install --packages-select challenge2
```



## Open another terminal in the same docker container

`docker exec -it --user 1000 CONTAINER_NAME bash`
and source ROS2 if needed:
`source /opt/ros/humble/setup.bash`

Or alternatively:
`docker compose run --rm --no-deps demo bash`



## Launch the filtering node

```bash
ros2 launch challenge2 lidar_filtering_node_launch.py
```



## Launch the clustering node
```bash
ros2 launch challenge2 clustering_node_launch.py
```



## Launch the polygon coordinates visualization node
```bash
ros2 launch challenge2 polygon_marker_launch.py
```



## Call the dump centroids service
Run the command:
```bash
ros2 service call /dump_centroids ch2_msg_srv/srv/DumpCentroids "{centroid: true}"
```



## Useful commands

- To inspect the TF tree

```bash
ros2 run rqt_tf_tree rqt_tf_tree 
```

- To print a specific TF

```bash
ros2 run tf2_ros tf2_echo robot_base_footprint robot_base_link
```



## GT obstacle positions

They can be found in the file:
```
fira_hackathon_workspace/src/romea_ros2/demos/fira_hackathon_gazebo/worlds/farm_vineyard_crops_challenge2.world
```



## Command to register the bag for the evaluation
Connect to the docker container and run the command:
```bash
ros2 bag record /robot/localisation/filtered_odom /evaluation/obstacle_detection -o evaluation_chal2_bag
```