# LiDAR Decision: RPLidar C1 vs A2 — for the NIDAR AirMouse drone

Analysis of the two datasheets against what we've actually built (Cartographer 2D SLAM
reading a single `/scan`) and the mission (dim indoor maze, 1 m corridors, ~15 m arena,
light matte-fabric walls, weight-constrained drone with prop guards).

> Note: the A2 datasheet on file is the older **A2M8** (8000 samples/s). The part you'd
> actually buy is the **A2M12** (16000 samples/s, 12 m). Where it matters, both are noted.
> The C1 datasheet is model **C1M1**.

## Head-to-head (from the datasheets)

| Spec | **RPLidar C1** | **RPLidar A2 (A2M8 → A2M12)** | Who wins for us |
|---|---|---|---|
| Ranging tech | **DTOF** (direct time-of-flight) | Triangulation (+OPTMAG brushless) | **C1** — flatter, more robust |
| Range (white 70%) | 12 m | 8 m (A2M8) → 12 m (A2M12) | tie |
| Range (black 10%) | **6 m** | not specified (triangulation weak on dark) | **C1** |
| Accuracy | **±30 mm, flat across range** | <1% of distance (degrades with range) | **C1** for 1–5 m walls |
| Sample rate | 5,000/s | 8,000/s → **16,000/s (A2M12)** | **A2** (denser map) |
| Angular resolution | 0.72° | 0.9° (A2M8) → **~0.22° (A2M12)** | **A2** |
| Scan rate | 8–12 Hz (10 typ) | 5–15 Hz (10 typ) | tie |
| Ambient light limit | **40,000 lux** (stated) | not stated (triangulation more light-sensitive) | **C1** |
| Weight | **~110 g** | ~190 g | **C1** (big deal on a drone) |
| Price (India) | ~₹11–14k | ~₹19,950 | **C1** |
| Eye safety | Class 1 (905 nm) | Class I (785 nm) | tie |
| Our software | `rplidar_ros`, same `/scan` | `rplidar_ros`, same `/scan` | tie — **zero code change** either way |

## What actually matters for THIS build

1. **Weight is the dominant factor.** On a ~2 kg drone squeezing through 1 m corridors with
   full prop guards, 80 g saved improves stability, endurance, and agility. **C1.**
2. **DTOF > triangulation indoors.** C1 gives flat ±30 mm accuracy at any distance and is more
   robust to lighting and to darker surfaces (6 m on black). Triangulation (A2) is most precise
   very close but degrades with distance and is more ambient-light sensitive. Our walls are
   1–5 m away → C1's flat accuracy fits better. **C1.**
3. **A2's one real edge — sample density — doesn't help our score.** The mapping score is about
   correct occupancy of **1 m grid cells**, not sub-cm geometry. Both resolve 1 m cells trivially,
   so A2M12's denser scan is nice-to-have, not needed.
4. **Same code.** Both use `rplidar_ros` and publish the same `/scan` LaserScan our Cartographer
   pipeline already consumes. Switching later is a driver launch arg, not a rewrite.

## Precedent
No public NIDAR team build writeups were found (niche/newer competition). Broader indoor
GPS-denied-drone practice: 2D SLAM (Cartographer/Hector/RTAB-Map) on **RPLidar A1/A2** or the
heavier **Hokuyo UTM-30LX**; lighter craft increasingly use the newer low-cost DTOF units. Our
reference repo `ahmedeltaher/Autonomous-drone-navigation` uses lidar + optical-flow fusion.

## VERDICT — order the **RPLidar C1**
Best fit for a weight-constrained indoor SLAM drone: lighter, cheaper, flat DTOF accuracy,
better lighting/surface robustness, and identical software to what we built. Choose the A2M12
only if you later want the densest possible map and weight/cost stop mattering — not our case.
