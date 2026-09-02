-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

-- /* Author: Darby Lim */

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.min_range = 0.15
-- Maze is 13.5m across and the real RPLidar C1 we're buying does 12m (6m on
-- dark surfaces). 20m just accepted long noisy returns that won't exist on the
-- real sensor; 10m keeps sim honest about hardware.
TRAJECTORY_BUILDER_2D.max_range = 10.
-- WAS 15m: every ray that returned no hit carved 15m of FREE space, which in a
-- 13.5m maze punches straight through walls and erases them -> washed-out,
-- unclear walls. 5m (the Cartographer default) keeps free-space carving local.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.
-- OFF (2026-09-02). This was toggled on twice to fix an apparent "map drift at
-- turns" -- but that diagnosis was WRONG. The map wasn't drifting: the drone was
-- flying OUT of the maze through the entry gap and legitimately mapping the open
-- ground outside, because the frontier explorer had no arena bounds. That's now
-- fixed there. So the IMU was solving a problem that didn't exist, while
-- historically correlating with EKF instability. Back to the config that flew
-- the maze cleanly.
--
-- Worth revisiting ONLY if real turn-drift shows up once exploration is properly
-- bounded -- and then as an isolated change, verifying const_pos_mode stays
-- false. (vision_pose_bridge's pose-jump guard stays regardless; it's good
-- protection either way.)
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

-- Motion filter: these were TurtleBot3 values. A wheeled robot sits perfectly
-- still, so a 0.1 DEGREE threshold was fine there. A hovering drone always
-- jitters more than 0.1deg, so EVERY scan became a new node -> node count
-- explodes -> the pose graph re-optimises constantly -> all submaps shift ->
-- the map visibly jumps/blinks. Back to Cartographer defaults, which suit a
-- drone: insert a node on 1deg of rotation, 0.2m of travel, or every 5s.
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.0)
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.2
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 5.

-- Slightly larger submaps than the default 90: fewer submap boundaries in a
-- small arena means fewer seams and a steadier map.
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 120

POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

-- Every optimisation pass shifts submaps, which is what you SEE as the map
-- jumping. With the motion filter fixed above, nodes accumulate at a sane rate,
-- so the default cadence is no longer constant. Raise this if it still jumps
-- too often (costs some drift correction); lower it for tighter accuracy.
POSE_GRAPH.optimize_every_n_nodes = 90

return options
