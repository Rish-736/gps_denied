#!/usr/bin/env python3
"""
NIDAR AirMouse - Drone SLAM bringup.

Starts (all with use_sim_time) everything on the ROS side of the drone-SLAM
pipeline in one shot:
  - gz -> /scan   LiDAR bridge
  - gz -> /clock  sim-time bridge   (required, else Cartographer drops scans)
  - base_link -> link static TF
  - cartographer_node (drone_2d.lua)
  - cartographer_occupancy_grid_node  (-> /map)
  - rviz2            (optional: use_rviz:=false to skip)
  - MAVROS           (optional: mavros:=true to enable flight control)

It does NOT start PX4/Gazebo itself - the run_drone_slam.sh wrapper does that,
because PX4 is a separate build system launched with `make`.

Run standalone:
  ros2 launch ~/nidar_ws/src/drone_cartographer/launch/drone_slam.launch.py
  ros2 launch ... world:=walls mavros:=true use_rviz:=false
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

PKG_DIR = os.path.expanduser('~/nidar_ws/src/drone_cartographer')
CONFIG_DIR = os.path.join(PKG_DIR, 'config')
RVIZ_CONFIG = os.path.join(PKG_DIR, 'rviz', 'drone_slam.rviz')
SCRIPTS_DIR = os.path.join(PKG_DIR, 'scripts')


def launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration('world').perform(context)
    use_rviz = LaunchConfiguration('use_rviz')
    use_mavros = LaunchConfiguration('mavros')

    scan_gz = (f'/world/{world}/model/x500_lidar_2d_0/link/link'
               f'/sensor/lidar_2d_v2/scan')
    imu_gz = (f'/world/{world}/model/x500_lidar_2d_0/link/base_link'
              f'/sensor/imu_sensor/imu')

    nodes = [
        # gz -> /scan
        Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name='lidar_bridge',
            arguments=[f'{scan_gz}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
            remappings=[(scan_gz, '/scan')],
            output='screen',
        ),
        # gz -> /imu  (Cartographer motion prior)
        #
        # TurtleBot3 fed Cartographer wheel odometry, which is why its map was
        # steady. The drone has no wheels, so Cartographer was estimating motion
        # from scan-matching alone -> every correction jumped base_link -> the
        # map "blinks". The IMU gives it that missing motion prior plus gravity
        # alignment (important: a drone tilts to move, which skews a 2D scan).
        #
        # Bridged from Gazebo rather than MAVROS on purpose: gz stamps sim time
        # (matching /scan, so no time-domain conflict) and it's a raw sensor, so
        # it can't form a feedback loop with vision_pose_bridge the way PX4's
        # own EKF2 odometry would.
        Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name='imu_bridge',
            arguments=[f'{imu_gz}@sensor_msgs/msg/Imu[gz.msgs.IMU'],
            remappings=[(imu_gz, '/imu')],
            output='screen',
        ),
        # gz -> /clock  (sim time)
        Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name='clock_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen',
        ),
        # base_link -> link (LiDAR scan frame)
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='base_to_lidar',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'link'],
            parameters=[{'use_sim_time': True}],
        ),
        # Cartographer
        Node(
            package='cartographer_ros', executable='cartographer_node',
            name='cartographer_node',
            arguments=['-configuration_directory', CONFIG_DIR,
                       '-configuration_basename', 'drone_2d.lua'],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
        # Occupancy grid -> /map
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
        # rviz2 (optional)
        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', RVIZ_CONFIG],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(use_rviz),
            output='screen',
        ),
    ]

    # MAVROS (optional - for flight control). tf.send stays false by default,
    # so it does NOT fight Cartographer for the TF tree.
    mavros_launch = os.path.join(
        get_package_share_directory('mavros'), 'launch', 'px4.launch')
    nodes.append(
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(mavros_launch),
            launch_arguments={
                'fcu_url': 'udp://:14540@127.0.0.1:14557'}.items(),
            condition=IfCondition(use_mavros),
        )
    )

    # Safety chain (only meaningful once MAVROS is up): feed PX4 a real
    # position (vision_pose_bridge), watch it against a local geofence
    # (geofence_monitor), and run the mission timer/entry-point memory
    # (mission_fsm). See docs/mission_rules_crucial_risks.md #1/#3/#5/#6.
    for script in ('vision_pose_bridge.py', 'geofence_monitor.py', 'mission_fsm.py'):
        nodes.append(ExecuteProcess(
            cmd=['python3', os.path.join(SCRIPTS_DIR, script)],
            name=script,
            output='screen',
            condition=IfCondition(use_mavros),
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='walls',
                              description='Gazebo world name (sets the /scan gz topic path)'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='Launch rviz2 with the preset config'),
        DeclareLaunchArgument('mavros', default_value='false',
                              description='Also launch MAVROS for flight control'),
        OpaqueFunction(function=launch_setup),
    ])
