# PX4 EKF2 Vision-Pose Fusion — Setup & Verification

Closes risk **#1** from `mission_rules_crucial_risks.md`: PX4 had no position estimate at
all indoors. This feeds Cartographer's pose into PX4's own EKF2 so PX4-side failsafes
(geofence, RTL/emergency-recall) have something real to work with.

Params below were read directly from this PX4 build's source
(`~/PX4-Autopilot/src/modules/ekf2/params_external_vision.yaml`, `params_gnss.yaml`,
`module.yaml`) — not guessed from memory, since older PX4 versions use a different param
(`EKF2_AID_MASK`) that does **not** exist in this build.

## What changed in software (already done, no action needed)
- New node `vision_pose_bridge.py` — reads the `map -> base_link` TF that Cartographer
  publishes and republishes it as `/mavros/vision_pose/pose` (a `PoseStamped`), which MAVROS
  forwards to PX4 as `VISION_POSITION_ESTIMATE`.
- Wired into `drone_slam.launch.py` — starts automatically whenever you run
  `./run_drone_slam.sh --mavros`.
- Verified in isolation (fake pose test, no sim needed): starts cleanly, correctly detects
  "TF not ready" before Cartographer is up, and publishes correctly once TF exists.

## What YOU need to do — set 3 PX4 parameters (one-time per SITL session)

Run the sim (`./run_drone_slam.sh --mavros`), then in a terminal:

```bash
source /opt/ros/jazzy/setup.bash

# MAVROS runs one ROS2 node per plugin (confirmed earlier in this project --
# see mission_rules_crucial_risks / hard-won-lessons). FCU parameters like
# EKF2_* live on the /mavros/param node and are set via the plain ROS2
# parameter API -- that's the mechanism MAVROS 2's own ParamSetV2 service
# docstring says to prefer. Each `param set` sends a MAVLink PARAM_SET to
# PX4 and mavros updates the ROS2-side value once the FCU acks it.

# 1) Tell EKF2 to fuse vision position (horizontal + vertical) + yaw.
#    Bitmask: bit0=horizontal pos, bit1=vertical pos, bit3=yaw -> 1+2+8 = 11
ros2 param set /mavros/param EKF2_EV_CTRL 11

# 2) Turn OFF GPS fusion so it doesn't fight vision.
#    (There is no real GPS at the venue anyway — this makes sim match reality.)
ros2 param set /mavros/param EKF2_GPS_CTRL 0

# 3) Height reference: since GPS is now off, EKF2 needs a different height source.
#    Use Baro (0) for now. Once the TF-Luna rangefinder is wired in on real hardware,
#    switch this to Range sensor (2) -- it's far less drift-prone indoors.
ros2 param set /mavros/param EKF2_HGT_REF 0
```

**Verify each one actually reached the FCU** (don't just trust the `set` command succeeded —
read it back):
```bash
ros2 param get /mavros/param EKF2_EV_CTRL   # should read back 11
ros2 param get /mavros/param EKF2_GPS_CTRL  # should read back 0
ros2 param get /mavros/param EKF2_HGT_REF   # should read back 0
```
If a readback doesn't match what you set, the FCU likely rejected it (e.g. wrong node name,
or mavros/PX4 not fully connected yet — check `ros2 topic echo /mavros/state --once` shows
`connected: true` first).

`EKF2_HGT_REF` is marked `reboot_required` in the PX4 source — so after setting it, restart
the SITL instance (Ctrl-C the PX4 konsole, rerun `./run_drone_slam.sh --mavros`) for it to
fully take effect. Steps 1 and 2 don't require a reboot.

## How to verify it actually worked

```bash
# A) vision_pose_bridge is actually publishing (should show live map-frame numbers)
ros2 topic echo /mavros/vision_pose/pose --once

# B) PX4/EKF2 is actually USING vision as a position source, not just receiving it
ros2 topic echo /mavros/estimator_status --once
# look for the flags corresponding to "ev_pos", "ev_yaw" etc. being set/true

# C) The proof that matters: PX4's own local_position should now track the SLAM map
#    Fly around a bit (as we did before) and confirm this still moves sensibly:
ros2 topic echo /mavros/local_position/pose
```

If (A) shows no data: check Cartographer is actually running and TF `map -> base_link`
exists (`ros2 run tf2_ros tf2_echo map base_link`) before the bridge can publish anything.

If (C) looks wrong/jumpy after this change: the most likely cause is `EKF2_EVP_NOISE`
(vision position measurement noise, default 0.1m) not matching Cartographer's real accuracy
— can be tuned later, not a blocker for first test.

## Report back
Run steps A-C after setting the params and tell me what you see — especially whether
`estimator_status` shows vision aiding flags active. That confirms risk #1 is actually
closed, not just wired.
