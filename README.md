# gps_denied — NIDAR AirMouse (Mission 2)

Software for a GPS-denied autonomous indoor drone (NIDAR 2026-27, Mission 2 — AirMouse).
Team Ardra. Platform: Ubuntu 24.04 + ROS 2 Jazzy + PX4 SITL + Gazebo Harmonic.

This repo holds the **software workspace** (`src/`) plus launch tooling. Large external
builds (PX4-Autopilot, turtlebot3_ws) are **not** vendored here.

---

## Current status

| Piece | State |
|---|---|
| ROS 2 Jazzy + PX4 SITL + MAVROS | ✅ working |
| Cartographer 2D SLAM on TurtleBot3 (reference) | ✅ working |
| Nav2 → drone velocity bridge (concept) | ✅ proven (x moves, z held in OFFBOARD) |
| **Drone LiDAR in Gazebo (`/scan`)** | ✅ **fixed** — see gotcha below |
| **Cartographer on the drone** | ✅ builds a map from real `/scan` data |
| One-command launcher | ✅ `run_drone_slam.sh` |
| Nav2 install | ⏳ in progress |
| Frontier exploration → YOLO → tagging → FSM → GCS | ⬜ next |

---

## Repo layout

```
src/drone_cartographer/
  config/drone_2d.lua          # Cartographer config tuned for the drone frames
  launch/drone_slam.launch.py  # starts all ROS nodes at once
  rviz/drone_slam.rviz         # rviz preset (map frame, /map, /scan)
run_drone_slam.sh              # one command: sim + SLAM + rviz
docs/                          # full technical handover doc
```

## How to run the drone SLAM sim (one command)

```bash
./run_drone_slam.sh            # sim + SLAM + rviz
./run_drone_slam.sh --mavros   # also start MAVROS (to fly)
./run_drone_slam.sh --no-rviz  # skip the rviz window
```

Opens PX4+Gazebo in its own konsole (the `pxh>` console stays usable), waits for the
world, then launches the bridges, static TF, Cartographer, occupancy grid, and rviz.
**Ctrl-C** in the launching terminal cleanly tears the whole stack down.

To fly (after `--mavros`), in the `pxh>` console — **in this order**, or the drone tips over:
```
commander arm
commander takeoff          # wait until it is hovering stably
# only THEN stream velocity setpoints / switch to OFFBOARD
```

---

## Hard-won gotchas

- **The `gz` PATH shadow (this cost weeks).** Sourcing ROS 2 puts a *broken* ROS-vendored
  `gz` (`/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz`) ahead of the real `/usr/bin/gz`.
  PX4 then fails to start Gazebo (`cannot find any available 'gz' command`) → the world/
  sensor never comes up → the LiDAR looked like it returned "all `.inf`". **Fix:** make the
  real gz win — `export PATH=/usr/bin:$PATH` (already baked into `run_drone_slam.sh`).
  This was misdiagnosed as a GPU/software-rendering bug; it never was.
- **Cartographer needs `/clock` bridged** when `use_sim_time:=true`, or it silently drops
  every scan. The launch file bridges `/clock` alongside `/scan`.
- **Flight command order:** never send horizontal velocity before takeoff+hover, or the
  drone flips on the ground.
- A flipped/tumbling drone makes the map "flash" in rviz — that's the sensor tumbling,
  not a SLAM bug.

See `docs/` for the complete technical handover (hardware, mission rules, full roadmap).
