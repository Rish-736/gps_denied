# Crucial Risks — NIDAR AirMouse

Living tracker. Found by cross-checking our built pipeline against the mission rules
(`mission_rules_compliance.md`). Update the **Status** line under each risk as it's fixed —
keep this current rather than adding a separate changelog.

Software-fixable risks (#1, #3, #5, #6) were tackled first since they need no hardware.
Hardware/testing risks (#2, #4, #7, #8, #9) are deferred until parts arrive.

---

## 🔴 #1 — PX4 itself doesn't know the drone's position
Cartographer knows where the drone is; PX4 never did — it just obeyed velocity commands
blind. Rule-required failsafes (geofence breach, emergency recall) need PX4 to have a real
position estimate, or they can't function no matter how good the software above them is.

**Status: ✅ Fixed (software) — pending your live sim confirmation**
- Built `vision_pose_bridge.py`: reads Cartographer's `map -> base_link` TF and republishes
  it as `/mavros/vision_pose/pose`, which MAVROS forwards to PX4 as an external vision
  position estimate.
- Found the exact PX4 parameters from this build's own source (not memory — an older param
  name, `EKF2_AID_MASK`, does NOT exist in this PX4 version): `EKF2_EV_CTRL=11`,
  `EKF2_GPS_CTRL=0`, `EKF2_HGT_REF=0`. Full setup + verification steps in
  `docs/px4_ekf2_vision_fusion_setup.md`.
- Verified in isolation with a fake-pose test (no sim needed): node starts cleanly, correctly
  reports "TF not ready" before Cartographer exists, publishes correctly once TF exists.
- **Still needed:** run it against the real sim + set the 3 PX4 params + confirm
  `/mavros/estimator_status` shows vision aiding flags active (steps are in the setup doc).

## 🟠 #2 — Loss of signal / no feed on the GCS
Video + telemetry currently share one WiFi link. If it degrades, you could lose the abort
channel and the video feed at the same moment.

**Status: ⬜ Deferred — needs hardware**
Fix: a separate, dedicated low-bandwidth radio (e.g. 900MHz/433MHz telemetry pair) purely for
command/abort, independent of the WiFi video link. Add to hardware order.

## 🟠 #3 — PX4's geofence doesn't work indoors (it's GPS-based)
PX4's built-in geofence assumes GPS lat/lon, which doesn't exist at the venue.

**Status: ✅ Fixed (software) — pending your live sim confirmation**
- Built `geofence_monitor.py`: watches `/mavros/vision_pose/pose`, records the first pose as
  the local origin, and flags a breach if the drone leaves a configurable box
  (`half_size_m`, default ±9m; `max_height_m`, default 3m) around it.
- **Verified end-to-end with a real integration test** (fake `ros2 topic pub` poses, no sim
  needed): pose at origin → correctly recorded as entry point. Pose 1m off → correctly
  ignored (inside bounds). Pose 5m off with a 2m test boundary → correctly flagged
  `GEOFENCE BREACH` and published `true` on `/mission/geofence_breach`.
- Wired into `drone_slam.launch.py`, starts automatically with `--mavros`.

## 🟡 #4 — Metal wall frames could hurt both LiDAR and WiFi
Fabric panels are good LiDAR targets, but the metal frame poles/joints in the scan plane
could cause spurious close readings; the same metal framework plus WiFi congestion at the
event could weaken the video/telemetry link.

**Status: ⬜ Deferred — needs real hardware + venue testing**
Fix (when testing starts): mount LiDAR height to mostly hit fabric not frame lines; add a
simple outlier filter on `/scan`; put the dedicated safety radio (#2) on 900MHz/433MHz to
dodge WiFi congestion.

## 🟡 #5 — "Return to exit" needs the drone to remember where it started
Cartographer's map has no built-in concept of "the exit" — entry and exit are the same point
per the rules, so the FSM must record and later return to it.

**Status: ✅ Fixed (software scaffold) — pending Nav2 integration + live confirmation**
- Built into `mission_fsm.py`: records the first pose it receives as the entry/exit point,
  publishes it on `/mission/exit_pose` when a return is triggered.
- **Verified in the same integration test as #3**: FSM correctly recorded entry at (0,0) and
  logged the transition to `EXPLORE`.
- **Still needed:** a Nav2 goal-sender that actually subscribes to `/mission/exit_pose` and
  drives the drone there — doesn't exist yet, comes with the Nav2/frontier-exploration work.
  Also still needed: long-duration drift testing (up to the full 30 min) since SLAM drift
  could move where "exit" actually is by the time the drone returns.

## 🟡 #6 — Exploring the whole maze could eat the entire time budget
Autonomous exit is scored as a strict, separate item. Inefficient exploration could still be
searching when the 30-minute cap hits, losing exit points even after finding survivors.

**Status: ✅ Fixed (software scaffold) — pending live confirmation**
- Built into `mission_fsm.py`: runs its own countdown (default 1500s / 25min, 5min margin
  under the real 30min cap) independent of exploration progress, and force-transitions
  `EXPLORE -> RETURN_TO_EXIT` when it expires — regardless of how much maze is left unseen.
- **Verified with a live test**: set budget to 8s, confirmed the FSM logged the countdown and
  correctly forced `RETURN_TO_EXIT` exactly when the budget hit zero.
- Same breach also triggers from the geofence monitor (#3) — either one forces the same safe
  state.

## 🟢 #7 — Vibration reaching the rigidly-mounted LiDAR
No gimbal means motor vibration transmits into the LiDAR and IMU, adding noise to Cartographer's
scan-matching.

**Status: ⬜ Deferred — needs hardware (real motors/frame)**
Fix: soft rubber vibration-damping mounts for flight controller + LiDAR; check PX4 vibration
metrics during a hover test before real mission runs.

## 🟢 #8 — Jetson overheating mid-mission, silently
Running SLAM + YOLO concurrently in an enclosed body for up to 30 minutes risks thermal
throttling exactly when detection matters most, and it's hard to notice live.

**Status: ⬜ Deferred — needs hardware (Jetson + enclosure)**
Fix: small heatsink/fan in the frame design; implement the "throttle detection rate first"
failsafe from the original roadmap (currently only planned, not built).

## 🟢 #9 — Sim-to-real gap is still completely untested
Everything proven so far is in Gazebo. Real fabric-on-metal walls, real RF conditions, and the
real LiDAR's actual behavior are unknowns until physically tested.

**Status: ⬜ Deferred — needs the real LiDAR to arrive**
Fix: hand-carry-test the LiDAR the moment it arrives (walk it around a room, watch the map
build) before the airframe is even built — cheapest way to catch a sim-to-real surprise early.
