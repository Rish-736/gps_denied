#!/usr/bin/env bash
# =====================================================================
# NIDAR AirMouse - one-command drone SLAM launcher.
#
#   ./run_drone_slam.sh                # sim + SLAM + rviz
#   ./run_drone_slam.sh --mavros       # also start MAVROS (to fly)
#   ./run_drone_slam.sh --no-rviz      # headless-ish (no rviz window)
#   WORLD=walls ./run_drone_slam.sh    # pick a different gz world
#
# Opens PX4+Gazebo in its own konsole window (so the pxh> console stays
# usable), waits for the world, then launches every ROS node at once.
# Ctrl-C in THIS terminal cleanly kills the whole stack.
# =====================================================================
# NOTE: no `set -u` — ROS 2's setup.bash references unbound vars by design.
set -o pipefail

WORLD="${WORLD:-walls}"
USE_RVIZ="true"
USE_MAVROS="false"
for arg in "$@"; do
  case "$arg" in
    --no-rviz) USE_RVIZ="false" ;;
    --mavros)  USE_MAVROS="true" ;;
    *) echo "Unknown option: $arg"; exit 1 ;;
  esac
done

# The gz PATH fix: make the REAL /usr/bin/gz win over the broken ROS shim.
export PATH=/usr/bin:$PATH
source /opt/ros/jazzy/setup.bash

LAUNCH=~/nidar_ws/src/drone_cartographer/launch/drone_slam.launch.py

echo ">> Cleaning up any stale sim/ROS processes..."
pkill -9 -f 'gz sim'                 2>/dev/null || true
pkill -9 -f 'bin/px4'                2>/dev/null || true
pkill -9 -f 'parameter_bridge'       2>/dev/null || true
pkill -9 -f 'cartographer'           2>/dev/null || true
pkill -9 -f 'static_transform_publisher' 2>/dev/null || true
pkill -9 -f 'mavros_node|mavros_router'                 2>/dev/null || true
sleep 2

cleanup() {
  echo; echo ">> Shutting down the whole stack..."
  pkill -9 -f 'gz sim' 2>/dev/null || true
  pkill -9 -f 'bin/px4' 2>/dev/null || true
  pkill -9 -f 'parameter_bridge' 2>/dev/null || true
  pkill -9 -f 'cartographer' 2>/dev/null || true
  pkill -9 -f 'static_transform_publisher' 2>/dev/null || true
  pkill -9 -f 'mavros_node|mavros_router' 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# 1) PX4 + Gazebo in its own konsole window (keeps the pxh> console available)
echo ">> Starting PX4 + Gazebo (world: $WORLD) in a new konsole..."
konsole --hold -e bash -c \
  "export PATH=/usr/bin:\$PATH; cd ~/PX4-Autopilot && PX4_GZ_WORLD=$WORLD make px4_sitl gz_x500_lidar_2d" &

# 2) Wait for the Gazebo world + drone model to actually exist
echo ">> Waiting for the Gazebo world to come up (up to ~2 min)..."
READY="false"
for i in $(seq 1 60); do
  if /usr/bin/gz topic -l 2>/dev/null | grep -q "world/$WORLD/model/x500_lidar_2d_0"; then
    READY="true"; break
  fi
  sleep 2
done
if [ "$READY" != "true" ]; then
  echo "!! Gazebo world did not come up. Check the PX4 konsole window for errors."
  exit 1
fi
echo ">> Gazebo is up. Starting the ROS SLAM stack..."
sleep 2

# 3) Launch every ROS node at once (bridges + TF + cartographer + occ grid + rviz [+ mavros])
ros2 launch "$LAUNCH" \
  world:="$WORLD" use_rviz:="$USE_RVIZ" mavros:="$USE_MAVROS"
