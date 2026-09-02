# PX4 files that live OUTSIDE this repo

These files belong in the PX4-Autopilot tree, which is a separate git repo and
is **not** vendored here. They are copied in so a new machine can reproduce the
exact flight configuration. **Copying them here does nothing on its own** — you
must install them into your PX4 checkout and rebuild.

## `airframes/4013_gz_x500_lidar_2d`

The x500 + 2D-LiDAR SITL airframe with every NIDAR-specific parameter baked in
(EKF2 external-vision fusion, GPS/RC failsafes disabled, indoor flight envelope
— slow speeds and gentle accel so the drone tracks tight maze corridors instead
of lunging into walls). See the comments in the file and
`../docs/px4_ekf2_vision_fusion_setup.md` for why each value is what it is.

Install it:

```bash
cp px4/airframes/4013_gz_x500_lidar_2d \
   ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4013_gz_x500_lidar_2d
# rebuild so the change lands in the build tree too
cd ~/PX4-Autopilot && make px4_sitl
```

If you edit params live in the `pxh>` console, mirror the change back into this
file so the config stays reproducible.
