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
| **Drone LiDAR in Gazebo (`/scan`)** | ✅ **fixed** — see gotcha below |
| **Cartographer on the drone** | ✅ **confirmed** — map grows live in rviz while flying |
| PX4 EKF2 external-vision fusion (SLAM pose → PX4) | ✅ working (`vision_pose_bridge`) |
| Auto arm → OFFBOARD → climb (no `pxh>` typing) | ✅ `auto_takeoff.py` |
| Nav2 planner + frontier exploration + position follower | ✅ autonomous maze flight |
| One-command launcher | ✅ `run_drone_slam.sh` |
| YOLO survivor detection → tagging → FSM → GCS | ⬜ next |

> **Known-good as of this branch:** the drone auto-takes-off, explores the maze
> by frontier, and returns to the entry/exit point. Speeds are tuned slow for
> tight corridors. See `docs/` for the tuning rationale.

---

## Repo layout

```
src/drone_cartographer/
  config/drone_2d.lua              # Cartographer config tuned for the drone frames
  config/nav2_drone.yaml           # Nav2 planner + costmaps (indoor-tuned)
  launch/drone_slam.launch.py      # SLAM layer: bridges, TF, Cartographer, rviz
  launch/drone_explore.launch.py   # autonomy layer: planner + explorer + follower
  scripts/
    vision_pose_bridge.py          # Cartographer pose -> PX4 EKF2 (map->base_link TF)
    auto_takeoff.py                # arm -> OFFBOARD -> climb, no pxh> typing
    frontier_explorer.py           # picks frontiers, asks planner for a path
    path_follower_position.py      # flies the path with PX4 POSITION setpoints
    generate_maze_world.py         # generates the test maze SDF worlds
run_drone_slam.sh                  # one command: sim + SLAM + rviz
px4/airframes/                     # PX4 airframe (install into ~/PX4-Autopilot, see px4/README.md)
docs/                              # technical handover + tuning rationale
```

### Architecture (autonomous flight)

Two layers so SLAM can be debugged without the autonomy stack on top:

```
run_drone_slam.sh  ->  PX4+Gazebo (sim)  +  drone_slam.launch.py (SLAM)
                                              |  publishes /scan, /map, map->odom->base_link TF
drone_explore.launch.py (autonomy)
   vision_pose_bridge     : map->base_link TF  -> PX4 EKF2 (so PX4 knows where it is)
   auto_takeoff           : arm -> OFFBOARD -> climb to altitude
   frontier_explorer      : /map -> pick frontier -> Nav2 planner -> /planned_path
   path_follower_position : /planned_path -> PX4 POSITION setpoints
```

We use **only Nav2's planner**, not its controller — the MPPI velocity
controller stalled the drone, so control is done with position setpoints. All
path reasoning happens in the `map` frame and is transformed into PX4's local
frame before publishing (mixing those two frames was what sent the drone out
of the maze).

## How to run the drone SLAM sim (one command)

```bash
./run_drone_slam.sh            # sim + SLAM + rviz
./run_drone_slam.sh --mavros   # also start MAVROS (to fly)
./run_drone_slam.sh --no-rviz  # skip the rviz window
```

Opens PX4+Gazebo in its own konsole (the `pxh>` console stays usable), waits for the
world, then launches the bridges, static TF, Cartographer, occupancy grid, and rviz.
**Ctrl-C** in the launching terminal cleanly tears the whole stack down.

## Fully autonomous maze run (no manual flying)

First-time setup: install the PX4 airframe (see [`px4/README.md`](px4/README.md)),
then two commands:

```bash
# terminal 1 — sim + SLAM + MAVROS
WORLD=nidar_maze_wide ./run_drone_slam.sh --mavros

# terminal 2 — autonomy: auto-takeoff, explore, return to entry
source /opt/ros/jazzy/setup.bash
ros2 launch src/drone_cartographer/launch/drone_explore.launch.py
```

The drone arms, climbs, explores the maze by frontier, and flies back to the
entry/exit point on its own. For the tighter `nidar_maze` world, pass its arena
bounds (see the launch file header). `lookahead:=0.4` slows it further for
sharp turns.

### Manual flying (optional, `--mavros`)

To fly by hand in the `pxh>` console — **in this order**, or the drone tips over:
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
- **MAVROS `local_position tf.send` MUST stay `false`.** Cartographer already owns the whole
  `map → odom → base_link` tree (`provide_odom_frame=true`, `published_frame=base_link`).
  If MAVROS also publishes `odom → base_link` (tf.send:true), the two fight over the same
  transform and the map blinks/jumps violently in rviz. Keep it false in px4_config.yaml.
- **A 2D map only grows when the drone TRANSLATES.** Yawing in place just re-scans the same
  walls — width/height stay fixed. Fly forward (OFFBOARD `linear.x`) to explore new area.
- **Flight command order:** never send horizontal velocity before takeoff+hover, or the
  drone flips on the ground.
- A flipped/tumbling drone makes the map "flash" in rviz — that's the sensor tumbling,
  not a SLAM bug.

See `docs/` for the complete technical handover (hardware, mission rules, full roadmap).
