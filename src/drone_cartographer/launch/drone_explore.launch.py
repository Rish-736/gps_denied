#!/usr/bin/env python3
"""
Autonomous exploration layer: Nav2 PLANNER + frontier explorer + POSITION follower.

Runs ON TOP of drone_slam.launch.py (which must already be providing /scan,
/map and the map->odom->base_link TF). Kept separate so SLAM can be debugged
without Nav2 in the way, and so this layer can be restarted alone.

  ros2 launch .../drone_explore.launch.py
  ros2 launch .../drone_explore.launch.py auto_takeoff:=false   # fly it up yourself

Architecture note: we use ONLY Nav2's PLANNER (planner_server), not its
controller. The MPPI velocity controller stalled the drone (commanded ~0 m/s
with a valid path). Instead:
   frontier_explorer  -> asks planner_server for an obstacle-free path
   path_follower_position -> flies that path with PX4 POSITION setpoints
So controller_server / behavior_server / bt_navigator / cmd_vel_to_mavros are
gone; only planner_server (+ its global costmap) remains from Nav2.

No map_server / AMCL: Cartographer already publishes /map and the TF tree.
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

PKG_DIR = os.path.expanduser('~/nidar_ws/src/drone_cartographer')
NAV2_PARAMS = os.path.join(PKG_DIR, 'config', 'nav2_drone.yaml')
SCRIPTS = os.path.join(PKG_DIR, 'scripts')

# Only the planner is managed now (it hosts the global costmap and the
# compute_path_to_pose action the explorer calls).
NAV2_NODES = ['planner_server']


def generate_launch_description():
    use_auto_takeoff = LaunchConfiguration('auto_takeoff')
    altitude = LaunchConfiguration('altitude')

    common = {'use_sim_time': True}

    nav2 = [
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen',
             parameters=[NAV2_PARAMS, common]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'use_sim_time': True,
                          'autostart': True,
                          'node_names': NAV2_NODES}]),
    ]

    arena = [
        '-p', ['arena_min_x:=', LaunchConfiguration('arena_min_x')],
        '-p', ['arena_max_x:=', LaunchConfiguration('arena_max_x')],
        '-p', ['arena_min_y:=', LaunchConfiguration('arena_min_y')],
        '-p', ['arena_max_y:=', LaunchConfiguration('arena_max_y')],
    ]

    explorer_and_follower = [
        # Coverage grid: tracks which arena cells the camera has actually
        # searched (not just mapped) and publishes /coverage_grid. The explorer
        # flies to unsearched cells until none remain -> full-arena search
        # guarantee. cell_size is a parameter (the brief doesn't fix it).
        ExecuteProcess(
            cmd=['python3', os.path.join(SCRIPTS, 'coverage_tracker.py'),
                 '--ros-args', *arena,
                 '-p', ['cell_size:=', LaunchConfiguration('cell_size')]],
            name='coverage_tracker', output='screen'),
        # Scores frontiers (distance vs information gain, with hysteresis) AND
        # unsearched coverage cells, asks the planner for a path to the winner,
        # publishes it on /planned_path. Arena bounds keep goals inside the maze.
        ExecuteProcess(
            cmd=['python3', os.path.join(SCRIPTS, 'frontier_explorer.py'),
                 '--ros-args', *arena],
            name='frontier_explorer', output='screen'),
        # Flies /planned_path with POSITION setpoints (also holds position at
        # altitude when idle -> serves as the OFFBOARD setpoint stream).
        # lookahead is the effective speed knob: PX4 turns that standing
        # position error into roughly MPC_XY_P * lookahead m/s.
        ExecuteProcess(
            cmd=['python3', os.path.join(SCRIPTS, 'path_follower_position.py'),
                 '--ros-args',
                 '-p', ['target_altitude:=', altitude],
                 '-p', ['lookahead:=', LaunchConfiguration('lookahead')]],
            name='path_follower_position', output='screen'),
        # arm -> OFFBOARD (setpoints already streaming from the follower),
        # no pxh> typing.
        ExecuteProcess(
            cmd=['python3', os.path.join(SCRIPTS, 'auto_takeoff.py'),
                 '--ros-args', '-p', ['takeoff_altitude:=', altitude]],
            name='auto_takeoff', output='screen',
            condition=IfCondition(use_auto_takeoff)),
    ]

    return LaunchDescription([
        DeclareLaunchArgument('auto_takeoff', default_value='true',
                              description='Automatically arm, take off and enter OFFBOARD'),
        DeclareLaunchArgument('altitude', default_value='1.2',
                              description='Flight altitude (must stay under the 2.5m walls)'),
        DeclareLaunchArgument('lookahead', default_value='0.4',
                              description='Carrot distance ahead on the path; '
                                          'sets cruise speed (~0.95 * lookahead m/s). '
                                          '0.4 -> ~0.38 m/s: slow & smooth keeps SLAM locked'),
        # Arena bounds, measured from the world SDF wall centre lines. Defaults
        # are nidar_maze_wide, inset slightly so goals sit off the walls.
        #   nidar_maze_wide  x[-8.75, 6.25]  y[-1.25, 13.75]
        #   nidar_maze       x[-7.50, 6.50]  y[-0.50, 13.50]
        # For nidar_maze pass: arena_min_x:=-7.25 arena_max_x:=6.25
        #                      arena_min_y:=-0.25 arena_max_y:=13.25
        DeclareLaunchArgument('arena_min_x', default_value='-8.5'),
        DeclareLaunchArgument('arena_max_x', default_value='6.0'),
        DeclareLaunchArgument('arena_min_y', default_value='-1.0'),
        DeclareLaunchArgument('arena_max_y', default_value='13.5'),
        # Coverage/survivor grid cell size. The brief does NOT fix this -- set it
        # to whatever the organizers confirm for the real arena.
        DeclareLaunchArgument('cell_size', default_value='1.0',
                              description='Coverage & survivor-report grid cell size (m)'),
        *nav2,
        *explorer_and_follower,
    ])
