#!/usr/bin/env bash
# =====================================================================
# NIDAR AirMouse - one-command full autonomy test (2.0m-corridor maze).
# Opens Gazebo (GUI) + RViz, brings up SLAM + MAVROS, then the autonomous
# explore/coverage/follower layer. Ctrl-C here tears the whole stack down.
#
#   ./run_nidar_test.sh
# =====================================================================
set -o pipefail
WORLD="nidar_sim"
export DISPLAY="${DISPLAY:-:0}"
export PATH=/usr/bin:$PATH
source /opt/ros/jazzy/setup.bash

echo ">> Cleaning stale sim/ROS processes..."
for p in 'gz sim' 'bin/px4' 'parameter_bridge' 'cartographer' \
         'static_transform_publisher' 'mavros_node|mavros_router' \
         'frontier_explorer|path_follower_position|coverage_tracker|auto_takeoff' \
         'vision_pose_bridge|geofence_monitor|mission_fsm|imu_monotonic' \
         'planner_server|lifecycle_manager'; do
  pkill -9 -f "$p" 2>/dev/null || true
done
sleep 3

cleanup() {
  echo; echo ">> Shutting down the stack..."
  for p in 'gz sim' 'bin/px4' 'parameter_bridge' 'cartographer' \
           'static_transform_publisher' 'mavros_node|mavros_router' \
           'frontier_explorer|path_follower_position|coverage_tracker|auto_takeoff' \
           'vision_pose_bridge|geofence_monitor|mission_fsm|imu_monotonic' \
           'planner_server|lifecycle_manager'; do
    pkill -9 -f "$p" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

# 1) PX4 + Gazebo with GUI visible
echo ">> Starting PX4 + Gazebo (GUI), world=$WORLD ..."
( cd ~/PX4-Autopilot && PX4_GZ_WORLD=$WORLD PX4_GZ_HEADLESS=0 \
    make px4_sitl gz_x500_lidar_2d ) </dev/null &

# 2) Wait for the Gazebo world + drone model
echo ">> Waiting for Gazebo model (up to ~3 min)..."
READY="false"
for i in $(seq 1 90); do
  if /usr/bin/gz topic -l 2>/dev/null | grep -q "world/$WORLD/model/x500_lidar_2d_0"; then
    READY="true"; break
  fi
  sleep 2
done
[ "$READY" = "true" ] || { echo "!! Gazebo did not come up. Check the make output above."; exit 1; }
echo ">> Gazebo up. Starting SLAM + RViz + MAVROS..."
sleep 3

# 3) SLAM stack (bridges + TF + cartographer + occ grid + rviz + mavros + safety chain)
ros2 launch ~/nidar_ws/src/drone_cartographer/launch/drone_slam.launch.py \
  world:="$WORLD" use_rviz:=true mavros:=true &

# 4) Wait for /map and mavros
echo ">> Waiting for /map + MAVROS ..."
for i in $(seq 1 60); do ros2 topic list 2>/dev/null | grep -q '^/map$' && break; sleep 2; done
for i in $(seq 1 30); do ros2 topic list 2>/dev/null | grep -q '/mavros/state' && break; sleep 2; done
sleep 5

# 5) Autonomous explore layer (tuned for 2.0m corridors, half-width 1.0m)
echo ">> Starting autonomous explore/coverage/follower layer..."
ros2 launch ~/nidar_ws/src/drone_cartographer/launch/drone_explore.launch.py \
  arena_min_x:=-6.75 arena_max_x:=6.75 \
  arena_min_y:=-0.75 arena_max_y:=12.75 \
  cell_size:=2.0 \
  lookahead:=0.4 \
  robot_radius:=0.40 \
  repulsion_influence:=0.9 repulsion_gain:=0.5 repulsion_max:=0.35 \
  brake_distance:=0.55 \
  setpoint_lpf:=0.35 yaw_lpf:=0.20 &

echo ">> ALL UP. Watch Gazebo (drone) + RViz (live map). Ctrl-C here to stop everything."
wait
