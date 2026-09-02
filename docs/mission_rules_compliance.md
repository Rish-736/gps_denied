# Mission Rules Compliance — NIDAR AirMouse

Full rule-by-rule read of `Mission Brief - NIDAR AirMouse.pdf` (Track 1, all 10 sections),
cross-checked against what we've built and what's still needed. This doc is the source of
truth for "are we building the right thing" — refer back to it whenever a design decision
comes up.

---

## Section-by-section: rule → what it means for us → status

### 1. Overall Objective
| Rule | Implication | Status |
|---|---|---|
| Enter/exit via one designated point | FSM needs explicit ENTRY/EXIT states | ⬜ not built |
| Navigate corridors/turns/junctions/rooms | Nav2 + frontier exploration | ⏳ Nav2 installing |
| Detect ≤6 survivors, ID grid coord/box | YOLO + position→grid mapping | ⬜ not built |
| **Simultaneously** generate + **continuously display** 2D map on GCS **while flying** | Map must stream live, not post-hoc | ✅ Cartographer live map proven; ⬜ GCS display |
| Tag survivor location with a marker on the map | Overlay markers on the live map | ⬜ not built |
| GCS shows live camera feed throughout | Video stream to GCS | ⬜ not built |
| Shortest time, safe, autonomous, accurate | Drives FSM + failsafe design | ⏳ ongoing |

### 2. Drone Configuration & Weight
| Rule | Implication |
|---|---|
| **No commercial RTF/market-ready airframes** | Must be a custom-built frame — our plan (custom 7" frame, Pixhawk, own motors) is already compliant. Do NOT buy a pre-built "ready to fly" drone as a shortcut. |
| All-up weight ≤ **10 kg** | Our estimate is ~1.9–2.0 kg. **Huge headroom** — weight is not a hard constraint we're close to hitting. |
| Prop guards must **fully protect the propeller operating area** | Needs full-perimeter guards (a full ring/cage around each prop's disc), not a partial/minimal guard. Factor into frame + guard purchase. |

### 3. Arena, Grid & Mission Environment
| Rule | Implication |
|---|---|
| ≤15m×15m, modular grid, net-covered top | Matches our sim `walls` world design intent |
| Corridor ≥1m clear width, ≥8ft (2.44m) vertical clearance | Drone footprint must clear 1m width with margin; 8ft height means altitude drift is not the tight constraint — **width is** |
| Room size 2m×2m | Confirms room-scale for detection/tagging logic |
| **Layout not disclosed beforehand** | Confirms true frontier exploration is required — no pre-planned path/waypoints allowed even in principle |

**Note:** this document does **not** state a numeric grid-cell size (e.g. "1m×1m") anywhere.
Earlier notes assumed 1m cells from other material — that number should be **reconfirmed with
organizers**, since it's not in this brief. Our software should keep the grid-cell size as a
configurable parameter, not hardcoded.

### 4. Mission Planning, Execution & Autonomy — the strictest section
| Rule | Implication |
|---|---|
| 5-minute setup only | GCS + drone must boot/connect fast — worth timing this in testing |
| No GPS/GNSS reliance | Already the entire premise (Cartographer + PX4 EKF2 without GPS) ✅ |
| **Any** manual control/path correction/waypoint adjustment/tagging input = violation | FSM must have zero manual-recovery paths — every recovery (lost localization, stuck, low battery) must be an autonomous state, exactly as the handover doc already flagged |
| **No FPV goggles / no separate monitoring device** — supervision ONLY through the GCS screen | GCS is the *single* window into the mission — it must show everything (see Section 5) reliably |
| Operator may **only** start mission + trigger e-stop | Confirms GCS UI needs exactly two operator controls: Start, Abort |
| **All mapping/detection/data-transfer must happen DURING flight — zero post-flight processing time** | This is a hard real-time constraint: no "record and process after landing." Confirms the Jetson must run SLAM + YOLO **live onboard**, not batch. Reinforces choice of Jetson Orin Nano + TensorRT (not a weaker board that would need to defer processing). |

### 5. Indoor Mapping & Mission Planner (GCS) Requirements
GCS must display, live, **during flight**: mission status, camera feed, continuously-updating
2D map, identified corridors/rooms (*"wherever technically feasible"* — a soft requirement),
survivor grid box, tagged survivor markers, drone position on the map, mission progress.

→ This is a **big, currently-unbuilt module.** Confirms the GCS dashboard (item 6 on the
original roadmap) needs: live map render, live video render, drone pose overlay, survivor
marker overlay, and a status panel — all in one screen. Foxglove or a custom rosbridge web
page both satisfy this; Foxglove is faster to stand up and already shows map+TF+image+markers
out of the box.

### 6. Survivor Detection & Localisation
Confirms: **onboard sensing/processing only** (matches our camera+Jetson plan, not cloud
detection), and that "grid coordinate/box" is almost certainly a **virtual grid overlaid on our
own generated map** (computed from map resolution + origin), not a physical sensor — this is a
pure software task (map pixel → grid cell math), no extra hardware needed for it.

### 7. Communication & Network Constraint
No GSM/LTE/5G/public WiFi/internet/cloud. **Local WiFi (private, no internet)** between drone
and GCS is fine and is what we already planned — it is not "an external network," it's the
team's own local link. No change needed.

### 8. Team Deployment & Human Intervention
Max 2 people for setup, 1 operator for the whole mission, zero assistance from anyone else.
No hardware implication, but **the GCS UI must be usable single-handedly** — one laptop, one
person, must see and control everything.

### 9. Launch, Landing & Field Constraints
| Rule | Implication |
|---|---|
| Launch from a fixed **2ft × 2ft** (~61cm × 61cm) area, drone must not cross/remain outside it before start | **Hard footprint check**: our planned ~350mm frame + guards (~450–500mm span) fits comfortably inside 610mm × 610mm. ✅ No issue, but confirm final guard span stays under ~580mm to keep margin. |
| Must not touch walls/ceiling/panels/obstacles | Reinforces LiDAR-based obstacle avoidance + Nav2 costmap inflation radius tuning |

### 10. Safety & Failsafe Requirements — **the one gap in our current hardware plan**
Required: E-stop/abort, and failsafes for **low battery, loss of command-and-control link,
geofence breach, mission abort, emergency recall.**

**This exposes a real gap.** Our plan routes *both* video and telemetry over the same local
WiFi link. If that WiFi link degrades or drops (very plausible — a maze of metal-framed fabric
panels can multipath/attenuate 2.4/5GHz), we could lose the **command/abort channel and the
video feed at the same time** — which would make "loss of command and control link" failsafe
detection late or the emergency-abort command undeliverable exactly when it's needed most.

**Recommendation: add a small, separate, dedicated low-bandwidth radio link purely for
command/telemetry/abort**, independent of the WiFi video link (e.g. a standard telemetry radio
pair). This is cheap, low-risk, and directly closes a rule requirement (reliable e-stop /
loss-of-link detection) rather than depending on the same pipe as bulk video.

---

## Net changes to the hardware plan (from this reading)

1. **Weight is not a hard constraint (10 kg ceiling, we're at ~2 kg)** — so the earlier
   LiDAR weight argument (C1 vs A2) should be read as an **agility/endurance/1m-corridor
   maneuverability** argument, not "we must save weight to stay under a limit." Both LiDARs
   are fine on the weight budget; the case for the lighter one is about flight quality inside
   tight corridors, not survival within 10 kg.
2. **Prop guards need to fully enclose the propeller disc** — factor this into frame/guard
   selection, not just "has some guard."
3. **NEW: add a dedicated command/telemetry radio, separate from the WiFi video link**, to
   reliably satisfy the loss-of-link and emergency-recall failsafe requirements.
4. **No hardware needed for "grid coordinate" detection** — it's a software overlay on our own
   generated map.
5. **Confirm the 1m×1m grid-cell assumption with organizers** — this document doesn't state a
   number; don't hardcode it.
6. **GCS dashboard is a bigger, still-fully-unbuilt piece** than earlier notes suggested — it's
   the literal single interface the rules require the operator to work through. Prioritize it
   accordingly once exploration + detection exist.

## Everything else in the original hardware BOM (`docs/hardware_bom.md`) is unaffected and
still applies: LiDAR, camera, TF-Luna altitude sensor, UBEC, ESC, frame, props/guards.
