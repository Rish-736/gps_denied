# Hardware Bill of Materials — GPS-Denied Drone (NIDAR AirMouse)

Shopping list to build the real drone. Prices are **approximate (INR, Aug 2026)** — verify at
purchase; stock/price move. Links favour India retailers.

**Already owned (do NOT re-buy):** Jetson Orin Nano 8GB, Pixhawk 6X, 4× SunnySky 920KV motors,
4S 8000mAh LiPo.

---

## TIER 1 — the GPS-denied sensing core (order first; this is what the software needs)

### 1. 2D LiDAR — the mapping/localization backbone
Two good choices. **Recommended: RPLidar C1** for a drone (lighter, cheaper, newer DTOF tech).

| | RPLidar C1 *(recommended)* | RPLidar A2M12 *(heavier, denser)* |
|---|---|---|
| Weight | **110 g** | 190 g |
| Range | 12 m | 12 m |
| Samples/sec | 5,000 (DTOF) | 16,000 (triangulation) |
| Price | ~₹11,000–14,000 | ~₹19,950 |
| Buy | [Amazon.in](https://www.amazon.in/WayPonDEV-RPLIDAR-C1-Avoidance-Navigation/dp/B0CMTXV5RC) · [ThinkRobotics](https://thinkrobotics.com/products/slamtec-rplidar-c1-laser-ranging-sensor) | [Robokits](https://robokits.co.in/sensors/lidar-laser-rangefinders/rplidar-a2m12-360-laser-range-scanner-18m) · [Robu.in](https://robu.in/product/slamtec-rplidar-a2m12-360-degree-laser-scanner-kit-12m-range/) |

Why C1: 80 g lighter matters a lot on a ~2 kg drone in 1 m corridors with prop guards, and the
maze (walls < 5 m, 15 m overall) never needs A2's extra range or sample density — 1 m grid cells
resolve easily on either. Both use the same `rplidar_ros` driver → **zero code change** from what
we built. Pick A2M12 only if you want the densest possible map for the mapping-accuracy score.

### 2. LiDAR power — dedicated 5V (do NOT run off Jetson USB)
The LiDAR's ~2.5 A startup surge browns out the Jetson. Give it its own rail.
- **5V 3A UBEC/BEC** — ~₹300–500 — search "5V 3A UBEC" on [Robu.in](https://robu.in).

### 3. Camera — survivor detection + live feed (arena is DIM, so low-light matters)
**Recommended: Arducam 2MP IMX462 (Sony STARVIS, ultra-low-light, near-0-lux).**
- **USB 3.0 version** *(easiest — plug-and-play in ROS2, no Jetson driver hassle)* — ~₹6,000–8,000
  — [Arducam](https://www.arducam.com/arducam-2mp-imx462-manual-focus-usb-3-0-camera-module.html) (ships to India)
- **CSI / Raspberry-Pi version** *(India stock, needs Arducam driver setup on Jetson)* —
  [Zbotic](https://zbotic.in/product/arducam-2mp-ultra-low-light-starvis-imx462-motorized-ir-cut-camera-for-raspberry-pi/) · [Fab.to.Lab](https://www.fabtolab.com/arducam-b0333-raspberry-pi-ultra-low-light-camera-1080p-hd-wide-angle-pivariety-module-1-2-8inch-2mp-starvis-sensor-imx462-auto-gain-control-isp-cmos-color)
- Add a **small onboard LED** for extra fill light.

### 4. Downward rangefinder — reliable altitude for PX4 (barometer drifts indoors)
- **Benewake TF-Luna** (0.2–8 m, UART/I2C, feeds Pixhawk EKF2) — ~₹2,199 —
  [KitsGuru](https://kitsguru.com/products/benewake-tf-luna-micro-lidar-distance-sensor-for-iot-its-8m) · [Probots](https://probots.co.in/benewake-tf-luna-lidar-distance-sensor-8m-uart.html) · [Amazon.in](https://www.amazon.in/Benewake-TF-Luna-Single-Point-Ranging-Interface/dp/B086MJQSLR)

---

## TIER 2 — to actually get airborne

### 5. 4-in-1 ESC (to drive the 4 motors)
- **45A BLHeli-S 4-in-1, 2–6S** — ~₹2,500–4,000 —
  [Indian Robo Store](https://indianrobostore.com/product/45a-blheli-s-brushless-speed-controller-4-in-1-2-6s-brushless-esc-for-rc-drones) · [Robu Cyclone 45A](https://robu.in/product/cyclone-45a-blheil_s-esc/)

### 6. Frame — ~350 mm, sized for 7–8" props (fits 1 m corridors with guards)
- 7" cinelifter-class carbon frame — ~₹2,000–4,000 —
  [TechHobby](https://techhobby.in/products/cinelifter) · [Evelta](https://evelta.com/drone-components/drone-frame/)
- Pick one whose motor mounts match the SunnySky 920KV hole pattern.

### 7. Propellers + FULL prop guards (guards mandatory per rules)
- 7" props (e.g. Gemfan 7037-class) + full-circle guards — ~₹1,000–2,000.
- Mount the LiDAR on TOP with a fully unobstructed 360° plane (no arm/antenna in the scan plane).

---

## OPTIONAL — vertical blind-spot patch
- **2–3× VL53L1X ToF** (up/down; LiDAR only sees its horizontal plane) — ~₹500–800 each — Robu/Amazon.in.

---

## Rough budget (core to fly)
LiDAR (C1) + camera + TF-Luna + UBEC + ESC + frame + props/guards ≈ **₹26,000–36,000**
(most owned parts already covered).

## Suggested order of purchase
1. **LiDAR + UBEC first** — you can bench-test our SLAM stack *handheld* (LiDAR on the Jetson,
   walk it around a room) before the airframe is ready. Fastest path to real-world validation.
2. Camera + TF-Luna next.
3. Airframe (ESC/frame/props/guards) once the sensing is proven.
