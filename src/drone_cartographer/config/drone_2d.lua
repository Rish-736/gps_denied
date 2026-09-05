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
-- ON (2026-09-03). Re-enabled after headless flight tests showed the REAL
-- blocker: Cartographer's pose diverges mid-flight (SLAM and PX4's EKF drift
-- apart, then feed each other garbage -> runaway). Cause: with IMU off,
-- Cartographer matches scan-to-scan only, which is fragile on a jittery, moving
-- drone -- doubly so in a maze of look-alike parallel walls (scan aliasing).
-- The IMU gives the scan matcher a motion prior between scans, the standard fix
-- for airborne 2D SLAM.
--
-- The earlier "IMU caused EKF divergence" was a MISDIAGNOSIS: that instability
-- was the map<->PX4-local frame-mixing bug (fixed in the follower/explorer) and
-- the out-of-maze mapping (fixed by arena bounds), not Cartographer's use of
-- IMU -- which is internal to SLAM and independent of PX4's EKF. Kept honest by
-- the divergence failsafe in vision_pose_bridge, which now hovers/lands if SLAM
-- does go bad, instead of flying off lost.
TRAJECTORY_BUILDER_2D.use_imu_data = true
-- Trust the scan matcher, but let the IMU-propagated pose seed each match so a
-- single bad scan can't yank the estimate. These weights are Cartographer's
-- airborne-friendly defaults for 2D.
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.
-- The x500's IMU streams fast; a short gravity time-constant keeps attitude
-- tracking responsive without chasing noise.
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 1.

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

-- CONSERVATIVE LOOP CLOSURE (2026-09-03). Flight tests found the real cause of
-- the mid-flight divergence: Cartographer's pose graph made a WRONG loop
-- closure -- it matched the current scan to a look-alike corridor elsewhere in
-- the maze and snapped the whole trajectory ~13m to that wrong place, after
-- which everything downstream followed the bad pose into a runaway. A maze of
-- near-identical parallel corridors is exactly where scan aliasing fools loop
-- closure. The fixes below make it only accept HIGH-confidence constraints and
-- never match across the whole arena:
--   * higher min_score -> reject marginal matches (the wrong ones)
--   * shorter max_constraint_distance -> don't try to loop-close against far
--     submaps, which is where the aliased wrong matches came from
--   * tighter fast-correlative search window -> can't jump far to "find" a match
-- The arena is only ~15 m, so local scan matching + IMU already give good
-- odometry; we don't need aggressive global loop closure, and its downside
-- (catastrophic wrong snaps) is far worse than a little uncorrected drift.
-- TIGHTENED FURTHER for the uniform 2.0m-corridor NIDAR maze. A perfectly
-- regular grid of identical 2m cells is the worst possible case for scan
-- aliasing: every junction looks like every other junction. In the last full
-- run local matching held rock-solid for ~250s, then a WRONG loop closure
-- snapped the pose during the return traverse over already-mapped territory and
-- tumbled the drone. Since the arena is tiny and the flight slow, we lean almost
-- entirely on local scan-matching + IMU (whose drift over ~4 min is centimetres)
-- and accept only very high-confidence, nearby loop closures.
POSE_GRAPH.constraint_builder.min_score = 0.78              -- was 0.72
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.85  -- was 0.80
POSE_GRAPH.constraint_builder.max_constraint_distance = 5.  -- was 9. (nearby only)
POSE_GRAPH.constraint_builder.sampling_ratio = 0.2          -- test fewer candidate constraints
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 3.   -- was 4.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(15.)  -- was 20

-- GLOBAL LOOP CLOSURE DISABLED (local SLAM + IMU only). Long debugging showed
-- global optimisation is the recurring crash cause here: even in the varied maze
-- the pose graph still occasionally makes a WRONG loop closure and oscillates
-- the map ~0.5 m back and forth (offset seen bouncing 0.01<->0.67 m, 3 SLAM
-- losses), which lurches the drone into a tumble -- and it bites hardest on the
-- long, thorough far-corner search we actually want. In a small 14 m arena on a
-- short (~4 min) slow flight, local scan-matching + IMU drift is small, and the
-- varied (rooms+loops) maze gives strong local features, so we do NOT need
-- global loop closure and its catastrophic downside. Setting the optimise
-- cadence above a whole run's node count means optimisation never fires: pure,
-- snap-free local SLAM. (On real feature-rich hardware, loop closure can be
-- revisited; it is the perfectly-repeatable sim geometry that makes it toxic.)
POSE_GRAPH.optimize_every_n_nodes = 100000

return options
