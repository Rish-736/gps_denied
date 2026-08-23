# NIDAR AirMouse — Complete Technical Handover
### GPS-Denied Autonomous Indoor Drone — Full Project Context for Claude Code

> This document contains every technical detail, decision, working configuration, confirmed result, open problem, and debugging lesson from the entire development effort so far. Paste it into Claude Code as opening context to continue the project with zero loss of continuity.

---

## SECTION 0 — DEVELOPER PROFILE & WORKING STYLE

- 3rd-year Electronics & Communication Engineering (ECE) student, VIT Vellore. Vice Captain of Team Ardra (autonomous drone team).
- **Strong in hardware/embedded**: PCB design (KiCad), IC/analog design (Cadence Virtuoso + Spectre simulator), ESP32/STM32 embedded programming, sensor fusion & IMU experience, Python.
- **Learning the software side** (ROS2/SLAM/nav) hands-on through this project. Not a software expert yet — wants exact commands, step-by-step, and debugging of the specific error rather than restarts.
- Learns by doing → error → fix. **Verify, never assume** (guessing wrong about topic/param names and distro versions cost many hours).

### How Claude Code should operate
- **When anything breaks, FIRST run `ros2 node list`** to confirm every required process is actually alive. ~80% of "it's broken" moments were actually a required background process that silently died.
- Confirm real values (`ros2 topic list`, `ros2 topic info`, `ros2 param get`, read the actual file) before building on assumptions.
- Test in isolated stages — never trust a multi-piece chain until each link is verified alone.
- Claude Code can run commands and read files directly — use that to verify machine state live rather than trusting this snapshot (some processes may have been closed since).

---

## SECTION 1 — THE MISSION (NIDAR 2026-27, Mission 2: AirMouse)

Building the **software side** of an autonomous indoor drone.

**Requirements:**
- Autonomous drone enters a GPS-denied maze via a designated entry point.
- Fully autonomous navigation of corridors/rooms — **NO GPS, NO manual control, NO internet/cloud** — local comms only.
- Build a **live 2D map** displayed on a Ground Control Station (GCS).
- Detect up to **6 survivors** (seated/crouched humans on the floor) and tag each on the map by **1m × 1m grid cell**.
- Show live camera feed + map + survivor tags on the GCS.
- Exit through the designated point. Max 30 min. Full propeller guards mandatory.

**Arena (confirmed from a real photo on the NIDAR site):**
- ~15m × 15m maze; **solid matte fabric/panel walls on black metal frames** (NOT glass — ideal LiDAR targets, this removes the one real LiDAR weakness).
- 1m-wide corridors; 2m × 2m rooms; netted top (blocks GPS).
- **Dim/moody lighting** (important: RGB camera needs good low-light perf and possibly an onboard LED; LiDAR is unaffected by lighting).
- Humans **seated/crouched on the floor** inside cells (important: train YOLO on seated humans viewed from above, not standing people).

**Scoring highlights:** 40 pts/survivor for correct grid-cell tagging (240 max), 220 pts for 2D grid mapping accuracy, autonomous exit is a discrete Yes/No scored item (50 pts), fast-completion bonus (25 pts), safe-completion bonus (15 pts). Total mission flight = 600 pts; Design Review 200; Business Strategy 200; Pre-Flight Inspection pass/fail gate. 1000 total.

**CRITICAL RULE — no manual override:** Manual intervention mid-mission (navigation input, path correction, waypoint change, survivor tagging, map correction, reset) is **banned and penalized −50 per instance**. The ONLY permitted operator actions are starting the mission and triggering an emergency safety-abort (which stops, doesn't recover). No FPV goggles or side monitors — supervision only through the GCS screen. **Therefore all mid-mission recovery (lost localization, stuck, low battery) MUST be autonomous states in the mission FSM.**

**Deadlines:** Registration closes **17 Aug 2026** (full registration — form + team details + fee; only the institutional approval letter can be submitted later). VEGAPilot/M3 concept proposal is separate, ~3rd week Sept 2026. Same person can't register for both M2 and M3, but VIT can field two separate teams.

---

## SECTION 2 — HARDWARE ARCHITECTURE (decided, with reasoning)

### Owned parts (locked in)
| Part | Weight | Notes |
|---|---|---|
| Jetson Orin Nano 8GB | ~130g | Companion computer — runs SLAM + YOLO concurrently |
| Pixhawk 6X (+ baseboard) | ~100g | Flight controller, running PX4 |
| Sunnysky 920KV motor ×4 | ~56g ea (224g) | X2212-class, F450/F550-class lift, designed for 9-10" props |
| 4S 8000mAh LiPo | ~650-750g | **CONFIRM exact weight** (real packs 630-1140g); heaviest single part, dominates CG |

### Parts still needed
- **Frame** — target ~350mm diagonal, sized for **7-8" props** (NOT F450/S500 — too big for 1m corridors with guards). Reasoning: smaller-than-max frame keeps corridor clearance; indoor hover doesn't need the motors' full 9-10" thrust. At 350mm + guards the span is ~450-500mm, leaving ~250mm clearance per side in a 1m corridor. Too-small frames actually wobble MORE (less inertia, more vibration coupling) — 350mm is the stability/clearance sweet spot.
- **Full propeller guards** (mandatory).
- **2D LiDAR — RPLidar A2M12** (0.2-12m, 360°, 16k samples/s, 10Hz, UART/USB, ~₹25k). **Primary mapping + localization backbone.** Confirmed suitable — matte fabric walls are ideal targets. **ELECTRICAL: needs a dedicated regulated 5V rail (NOT off Jetson USB) — ~2.5A startup surge will brown out the Jetson.** Hard-mount flat, NO gimbal (a gimbal breaks Cartographer's fixed sensor-to-body-frame assumption). Must sit on TOP with a fully unobstructed 360° horizontal scan plane — any arm/antenna in that plane becomes a permanent false wall in every map.
- **Camera — plain RGB (RPi cam / USB webcam), NOT depth.** Reasoning: LiDAR handles mapping/localization; the camera only does survivor detection + video feed (both 2D tasks). Survivor tagging is by coarse 1m grid cell, and the drone knows its own position from LiDAR, so "fly close, then tag current cell" hits the right cell without depth. Depth (OAK-D Lite, which also offloads YOLO onboard) is a possible LATER upgrade only if testing proves it's needed. Front-facing, slight downward tilt. Add a small onboard LED (dim arena).
- **4-in-1 ESC** (40A+, to suit these motors).
- **Downward 1D rangefinder** (TF-Luna / VL53L1X, ~$15-30) — RECOMMENDED for reliable altitude (barometers drift indoors from HVAC/propwash/doors). Feeds PX4 EKF2 as a trustworthy height source.
- **A few ToF sensors** (~₹400 ea) pointed up/down to patch the 2D LiDAR's vertical blind spot. Horizontal obstacle avoidance is fully handled by the LiDAR; ToF covers the out-of-plane gap cheaper/better than a depth cam. (8ft vertical clearance is arena-guaranteed, so we don't need to sense it.)
- **Telemetry/video:** stream compressed **H.264 video + ROS2 telemetry over a LOCAL WiFi link** (legal under the no-internet rule) — NOT a separate VTX system. Compress video before sending (raw saturates the link and starves telemetry). Keep video and telemetry on separate ports/channels. Confirm allowed WiFi bands with organizers (some comps restrict 2.4 vs 5GHz).

**Estimated total AUW: ~1.9-2.0 kg** (well under the 10kg limit).

### Optical flow — decided AGAINST
Some reference repos use a 1D lidar + optical flow. We add the **1D downward lidar (altitude)** but **skip optical flow**: its job (horizontal velocity without GPS) is already done by LiDAR scan-matching. Only revisit optical flow if real testing shows scan-matching struggling in large open rooms (a known LiDAR weak point).

---

## SECTION 3 — SOFTWARE STACK (decided)

- **OS/ROS:** Ubuntu 24.04 + **ROS2 Jazzy** (confirmed: Cartographer + TurtleBot3 both have Jazzy support).
- **Flight:** PX4 + MAVROS2.
- **SLAM:** **Cartographer (2D mode)** — right for low-texture, dim, corridor arena; visual SLAM (ORB-SLAM etc.) would fail there.
- **Navigation:** Nav2 + frontier exploration.
- **Detection:** YOLOv8 → exported to TensorRT for the Jetson. Train on seated humans in dim light from above.
- **GCS:** Foxglove or a custom rosbridge web dashboard.
- **Sim:** Gazebo (new `gz` sim) + PX4 SITL.

### The 12-module software build order
dev environment → simulation → sensor drivers → SLAM → localization fusion → autonomous exploration → survivor detection → survivor tagging → mission state machine → GCS dashboard → data logging → failsafes/watchdog.

---

## SECTION 4 — WHAT'S DONE AND CONFIRMED WORKING

1. **ROS2 Jazzy installed**; turtlesim tested and working.
2. **Workspaces on disk:**
   - `~/nidar_ws` — main workspace; contains `src/drone_cartographer/config/drone_2d.lua`.
   - `~/turtlebot3_ws` — TurtleBot3 sim, fully built.
   - `~/PX4-Autopilot` — PX4 SITL, built.
3. **PX4 SITL works** — `make px4_sitl gz_x500` arms/takes off/lands via the `pxh>` console.
4. **MAVROS connects to PX4 SITL** — `Got HEARTBEAT, connected. FCU: PX4 Autopilot`.
5. **★ Cartographer 2D SLAM CONFIRMED FULLY WORKING on TurtleBot3** — live map built in rviz2 while driving with `teleop_keyboard`. This proved the ENTIRE SLAM pipeline end-to-end on a reference robot. (Biggest milestone.)
6. **★ The Nav2→drone velocity bridge CONCEPT CONFIRMED WORKING (verified with real numbers):**
   - PX4 accepts horizontal velocity commands on **`/mavros/setpoint_velocity/cmd_vel_unstamped`** (type **`geometry_msgs/msg/Twist`**) in **OFFBOARD mode** while **independently holding altitude**.
   - Verified: streamed a constant `{linear: {x: 0.3}}` Twist at 20Hz, switched to OFFBOARD via `ros2 service call /mavros/set_mode ...`, and confirmed via `ros2 topic echo /mavros/local_position/pose` that **x position climbed steadily while z stayed flat**. Also confirmed `mode: OFFBOARD` held via `/mavros/state`.
   - This was THE key unproven assumption behind "make a drone behave like a 2D nav-stack robot with held altitude." **Now validated.**

### Extra dependencies that were needed (install if rebuilding)
`ros-jazzy-ros-gz`, `ros-jazzy-dynamixel-sdk`, `python3-colcon-common-extensions`, `ros-jazzy-cartographer`, `ros-jazzy-cartographer-ros`, `ros-jazzy-tf2-tools`.
TurtleBot3 walls world needs `GZ_SIM_RESOURCE_PATH` set to the `turtlebot3_gazebo` models folder.

---

## SECTION 5 — THE DRONE-SLAM CONFIG (exact working values)

**File:** `~/nidar_ws/src/drone_cartographer/config/drone_2d.lua`
Critical values (changed from TurtleBot3 defaults, which assumed `base_footprint`/`imu_link` frames that DON'T exist on the drone):
```lua
tracking_frame   = "base_link",
published_frame  = "base_link",
provide_odom_frame = true,      -- Cartographer builds the whole map->odom->base_link tree itself
use_odometry     = false,
-- use_imu_data = false
```
**Reasoning:** With `provide_odom_frame = true` + `published_frame = "base_link"`, Cartographer self-provides the entire TF tree from scan matching alone — it does NOT depend on MAVROS's TF. This deliberately avoids a **time-domain conflict**: MAVROS publishes transforms stamped in wall-clock time (~1.78e9), while the sim runs on sim time (starts from 0). Feeding MAVROS's TF into a sim-time Cartographer caused every scan to be dropped. So **MAVROS's `local_position tf.send` must be OFF** (`ros2 param set /mavros/local_position tf.send false`) so nothing competes.

**Correct LiDAR Gazebo topic (world-name-dependent!):**
- `default` world: `/world/default/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan`
- `walls` world: `/world/walls/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan`
- **Decoy path that carries NO data:** `.../lidar_sensor_link/sensor/lidar/scan` — do not use it.
- The scan's `frame_id` is `link`; static transform `base_link → link` bridges it to the body.

---

## SECTION 6 — THE CURRENT BLOCKER (parked, NOT blocking progress)

**Symptom:** The drone's LiDAR in Gazebo returns **all `.inf` ranges** — even with an obstacle only ~5m away, well inside the 30m sim range. So Cartographer has nothing to scan-match, never publishes `map → odom`, and `tf2_echo map odom` reports "two or more unconnected trees."

**Exhaustively ruled out (all confirmed healthy):** ROS2, PX4, MAVROS, the ros_gz_bridge, Cartographer config, the TF tree, `/scan` publishing at ~30Hz, sim-time settings. The scan message is well-formed (correct 270° arc, `range_max: 30.0`) — it just contains only `.inf`/`intensities`, no finite ranges.

**Best diagnosis:** `lidar_2d_v2` is a **`gpu_lidar`** sensor (moved from `gpu_ray` in the PX4 PR that added it). GPU-lidar computes range by RENDERING the scene, not physics collision. If Gazebo rendering falls back to software (`llvmpipe`) instead of a real GPU, the sensor reads an empty render → every ray returns max range → `.inf`. A real PX4 GitHub thread reports the identical "reports max distance regardless of obstacles" symptom on this exact sensor.

**Untested diagnostics to try in Claude Code (in order):**
1. `glxinfo | grep "OpenGL renderer"` — if `llvmpipe`, that's software rendering (likely root cause). Try `export GZ_SIM_RENDER_ENGINE=ogre` before relaunch.
2. `make px4_sitl gz_x500_lidar_front` — if it also `.inf`, confirms rendering issue; if it works, bug is specific to `lidar_2d_v2`.
3. In the Gazebo GUI, toggle "Visualize Lidar" on the sensor — see if rays are drawn at all / terminate on obstacles.
4. Inspect `~/PX4-Autopilot/Tools/simulation/gz/models/lidar_2d_v2/model.sdf` — check `<sensor type=...>`, `<pose>`, ray config.
5. Note: drone spawns at z≈2.5m; a flat 2D scan plane at 2.5m may sail over short obstacles — also check ranges near ground.
6. Always kill stale sims first (`pkill -9 -f gz; pkill -9 -f px4`) — two Gazebo instances (`/world/default/...` AND `/world/walls/...` topics) were running simultaneously at one point.

**★ STRATEGIC DECISION — do NOT let this block the project.** Everything downstream of `/scan` (SLAM, exploration, detection, tagging, mission FSM, GCS) is IDENTICAL whether the LiDAR sits on a drone or a TurtleBot3 — same topic, same map, same algorithms, same `cmd_vel` output. So **build the full mission pipeline on TurtleBot3 (which works), and treat the drone sensor bug as a separate low-priority investigation.** The real RPLidar A2 hardware physically measures (doesn't render), so it will very likely just work when it arrives.

---

## SECTION 7 — THE NAV2→DRONE BRIDGE (proven mechanism + what's left)

**Proven:** publishing `Twist` to `/mavros/setpoint_velocity/cmd_vel_unstamped` in OFFBOARD mode drives horizontal motion while PX4 holds altitude.

**PX4 gotcha (confirmed):** setpoints must ALREADY be streaming BEFORE switching to OFFBOARD, or the mode switch is rejected. `mode_sent=True` only means the request was acknowledged — always verify the mode actually held via `ros2 topic echo /mavros/state --once`.

**What's left to build (the bridge node):** A small ROS2 node (~60-80 lines Python) that sits between Nav2 and MAVROS:
1. Passes Nav2's horizontal velocity (`cmd_vel`) through untouched.
2. Reads current altitude and adds a small proportional vertical-velocity correction to hold a target height.
3. Publishes the combined `Twist` to `/mavros/setpoint_velocity/cmd_vel_unstamped`.
This is the same shape as `ahmedeltaher/Autonomous-drone-navigation`'s `mavsdk_offboard` approach. **Honest caveat:** altitude-hold-while-accepting-velocity is a known PX4 soft spot — there is NO perfect out-of-box library; every real project rolls its own. Test in stages. Also Nav2's velocity/acceleration limits (tuned for a ground robot) will likely need retuning for a hovering drone.

**Immediate next step (where we left off):** Wire Nav2's REAL `cmd_vel` output through the bridge for a simple point-to-point goal, on the EMPTY/simple world first (isolate "does Nav2's output drive the bridge" from "does it work in a maze"). Steps: confirm Nav2's actual velocity output topic (`ros2 topic list | grep cmd_vel` once Nav2 is up — don't assume plain `/cmd_vel`); remap it to `/mavros/setpoint_velocity/cmd_vel_unstamped`; send one manual goal; watch position numbers (x/y toward goal, z flat).

---

## SECTION 8 — USEFUL REPOS (reference / porting)

- **`DaniGarciaLopez/ros2_explorer`** — Cartographer + Nav2 + frontier exploration, complete pipeline, includes a **CSV-to-maze generator** (can build the NIDAR maze from a spreadsheet) and a single bringup launch file. Tested on **Humble — needs Humble→Jazzy porting** (Nav2/Gazebo API changes; uses Gazebo Classic not new `gz`, so the sim part needs the most work — but the Cartographer config, frontier algorithms, and CSV maze generator port easily). We started cloning it into `~/turtlebot3_ws/src` (needed `pip3 install pandas --break-system-packages`, then rosdep install, then colcon build — expect build errors to work one at a time).
- **`ahmedeltaher/Autonomous-drone-navigation`** — GPS-denied indoor drone, PX4/ROS2/MAVSDK; has `vision_pose_estimator` (SLAM→EKF2 fusion) and `mavsdk_offboard` (velocity/position setpoints). Good reference for the velocity bridge and vision-pose fusion.
- **`mertgulerx/frontier_exploration_ros2`** — frontier exploration written/tested for ROS2 **Jazzy** with Nav2 (our exact distro — least porting pain).
- **`monemati/PX4-ROS2-Gazebo-YOLOv8`** — YOLOv8 detection on PX4+Gazebo (detection module reference).
- **`AniArka/Autonomous-Explorer-and-Mapper-ros2-nav2`** — simple single-file frontier explorer (`explorer.py`), great for understanding the algorithm.
- **`rafaelmaeuer/Autonomous-Indoor-Drone`** — RPLidar A2 + ultrasonics (our exact sensor combo), ROS1/older — hardware-integration reference only.

---

## SECTION 9 — REMAINING WORK (prioritized roadmap)

**HIGH-PRIORITY housekeeping (do early):** Write a SINGLE ROS2 launch file that starts all processes together with consistent `use_sim_time:=true` everywhere. Manual multi-terminal juggling caused ~80% of debugging pain (processes silently dying between restarts). ~30 lines; turns the whole marathon into one command.

**Then, in order:**
1. **Wire Nav2 real `cmd_vel` through the bridge** — simple point-to-point goal, empty world first. (Immediate next step.)
2. **Frontier exploration** on TurtleBot3 (use `ros2_explorer` or `mertgulerx/frontier_exploration_ros2` as backbone). Tune for the fast-completion bonus.
3. **Survivor detection** — YOLOv8 on the RGB feed, trained on seated humans in dim light from above; export to TensorRT.
4. **Survivor tagging** — custom node: YOLO detection + drone position → correct 1m grid cell. **Confidence-gate:** only tag when BOTH detection confidence AND localization confidence are high; "fly close before tagging" logic to avoid false distant tags.
5. **Mission FSM** — `TAKEOFF → EXPLORE → (tag survivors along the way) → EXIT_CONDITION_MET → RETURN_TO_ENTRY → LAND`. Include autonomous recovery states (lost-localization hover+relocalize, stuck backoff, low-battery forced return) since manual override is banned (−50/instance). Every scored rulebook item should map to an explicit FSM state/transition.
6. **GCS dashboard** — Foxglove or custom rosbridge web. Must show (rulebook): mission status, live camera feed, 2D map, drone position, survivor tags, mission progress. Compress video; keep on separate port from telemetry. (There's a reference dashboard image on the NIDAR site to match.)
7. **Data logging** — `ros2 bag record` all key topics, auto from mission start.
8. **Failsafes/watchdog** — autonomous responses ranked flight-safety > mission-completion > scoring: comm-loss (continue autonomously), localization-loss (hover→relocalize→RTL), low-battery (forced return), collision-imminent (stop+backoff via ToF/costmap), thermal/CPU overload (throttle detection rate first). Then failure-injection test each one.

**Then:** move the whole pipeline from sim onto real hardware (Jetson + RPLidar A2 + RGB cam + Pixhawk 6X), plus the Nav2→MAVROS altitude-hold bridge node.

---

## SECTION 10 — HARD-WON DEBUGGING LESSONS (don't repeat these)

- **"Already running" errors** → `pkill -f px4`, `pkill -f gz`, `pkill -f cartographer`, `pkill -f mavros`. Stopped processes (state `T` in `ps`) need `kill -9 <PID>` — plain pkill won't clear them.
- **`static_transform_publisher` output appears on `/tf_static`, NOT `/tf`** — checking the wrong topic looks like failure.
- **`TF_OLD_DATA` errors** = stale/leftover sim session with old timestamps (often a paused/reset Gazebo, or a leftover model from a prior world) → full clean restart fixes it.
- **`use_sim_time` MUST use `--ros-args -p use_sim_time:=true` (DOUBLE dash).** Single dash `-ros-args` silently parses it as a remap; the param never takes effect. This cost hours — the giveaway was a `Found remap rule 'use_sim_time:=true'. deprecated` warning that looked harmless but wasn't.
- **Each MAVROS plugin is its OWN node** (e.g. `/mavros/local_position`, `/mavros/global_position`) — params live on the specific plugin node, not on `/mavros/mavros`. `ros2 node list` shows them all.
- **Frame names must match the actual robot** — the drone's scan is `frame_id: link`, tracking frame `base_link`; TurtleBot3's config assumed `base_footprint`/`imu_link` which don't exist on the drone. This silently made Cartographer drop every scan until the config was edited.
- **Empty world = `.inf` ranges = no map, and it is NOT a bug.** Always `ros2 topic echo /scan --once` and check `ranges:` before assuming the pipeline is broken.
- **An empty gray Gazebo world makes motion impossible to judge by eye** — use `ros2 topic echo /mavros/local_position/pose` and read x/y/z numbers instead.
- **PX4 requires setpoints streaming BEFORE switching to OFFBOARD**, or the switch is rejected. `mode_sent=True` ≠ mode held — verify with `ros2 topic echo /mavros/state --once`.
- **Close terminal windows fully when done** — don't leave processes lingering in a stopped state; that's what caused most stale-process issues.

---

## SECTION 11 — THE WORKING STARTUP SEQUENCES (reference)

### Drone-SLAM test (6 terminals — the sequence that gets all nodes alive)
- **T1:** `cd ~/PX4-Autopilot && PX4_GZ_WORLD=walls make px4_sitl gz_x500_lidar_2d` (wait for `pxh>`)
- **T2:** `ros2 run ros_gz_bridge parameter_bridge '/world/walls/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan' --ros-args -r /world/walls/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan:=/scan`
- **T3:** `ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link link`
- **T4:** `ros2 run cartographer_ros cartographer_node -configuration_directory ~/nidar_ws/src/drone_cartographer/config -configuration_basename drone_2d.lua --ros-args -p use_sim_time:=true`
- **T5:** `ros2 run cartographer_ros cartographer_occupancy_grid_node --ros-args -p use_sim_time:=true`
- **T6 (checks):** `ros2 node list` (confirm all present) → `ros2 topic hz /scan` (~30Hz) → `ros2 topic echo /scan --once` (**check `ranges:` are real, not `.inf`**) → `ros2 run tf2_ros tf2_echo map odom` (must stream numbers, not "unconnected trees")

### Velocity-bridge test (the proven one)
- **T1:** `cd ~/PX4-Autopilot && make px4_sitl gz_x500` (wait for `pxh>`)
- **T2:** `ros2 launch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"` (wait for `Got HEARTBEAT`)
- **T1 pxh> console:** `commander arm` then `commander takeoff`
- **T3 (stream setpoints FIRST):** `ros2 topic pub -r 20 /mavros/setpoint_velocity/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {z: 0.0}}"`
- **T4 (then switch mode):** `ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'OFFBOARD'}"`
- **Verify:** `ros2 topic echo /mavros/local_position/pose` → x increases, z flat = success.

---

## SECTION 12 — IMMEDIATE ASK FOR CLAUDE CODE

Pick up at **"wire Nav2's real `cmd_vel` through the confirmed velocity bridge for a simple point-to-point goal, on the empty/simple world."** But first: since you can run commands directly, **verify the live machine state** (`ros2 node list`, what's running, which workspaces built cleanly) rather than trusting this snapshot. Test in isolated stages, confirm before assuming, and when something breaks check all required nodes are alive first. Early on, help write the single consolidated launch file to end the multi-terminal pain. Keep building the pipeline on TurtleBot3 (proven) while the drone LiDAR `.inf` bug stays a separate low-priority investigation.
