#!/usr/bin/env python3
"""
Generate a NIDAR-AirMouse-spec maze world for PX4 / Gazebo Harmonic.

Why this exists: the stock PX4 `walls` world is 4 giant boxes -- nothing like
the competition arena -- and every off-the-shelf Gazebo maze we found is either
Gazebo Classic or has ~1m-tall walls, which a drone flying at ~1.2m simply
looks straight over (its 2D LiDAR plane clears the walls and sees nothing).

Built to the mission brief (Section 3):
  - arena <= 15m x 15m            -> default 14m x 14m
  - corridor clear width >= 1m    -> --corridor (default 1.5m, use 1.0 for exact spec)
  - standard room 2m x 2m         -> --rooms carves 2x2 open rooms
  - vertical clearance >= 8ft     -> walls 2.5m tall, so the drone cannot see over them
  - same designated entry+exit    -> one boundary gap, drone spawns there at origin
  - thin panel walls              -> 0.08m, matching fabric-on-metal-frame panels

The maze is carved with a recursive backtracker, so it is connected *by
construction*; a flood fill then re-verifies every open cell is reachable from
the entry before anything is written (a disconnected maze would strand the
drone and waste a whole test run).

Usage:
  python3 generate_maze_world.py                       # default 14m, 1.5m corridors
  python3 generate_maze_world.py --corridor 1.0        # exact competition spec
  python3 generate_maze_world.py --seed 7 --loops 0.15 # different layout
  python3 generate_maze_world.py -o ~/PX4-Autopilot/Tools/simulation/gz/worlds/nidar_maze.sdf
"""
import argparse
import os
import random

WALL_THICKNESS = 0.08   # thin panel, like the real fabric-on-frame walls
WALL_HEIGHT = 2.5       # > 8ft ceiling, so the drone can't out-climb the maze


def carve_maze(nx, ny, rng):
    """Recursive backtracker. Returns (vwalls, hwalls) sets of remaining walls.

    vwalls: (i, j) = vertical panel at x=i spanning y in [j, j+1]
    hwalls: (i, j) = horizontal panel at y=j spanning x in [i, i+1]
    """
    vwalls = {(i, j) for i in range(nx + 1) for j in range(ny)}
    hwalls = {(i, j) for i in range(nx) for j in range(ny + 1)}

    visited = [[False] * ny for _ in range(nx)]
    stack = [(0, 0)]
    visited[0][0] = True

    while stack:
        cx, cy = stack[-1]
        neighbours = []
        if cx > 0 and not visited[cx - 1][cy]:
            neighbours.append((cx - 1, cy, 'W'))
        if cx < nx - 1 and not visited[cx + 1][cy]:
            neighbours.append((cx + 1, cy, 'E'))
        if cy > 0 and not visited[cx][cy - 1]:
            neighbours.append((cx, cy - 1, 'S'))
        if cy < ny - 1 and not visited[cx][cy + 1]:
            neighbours.append((cx, cy + 1, 'N'))

        if not neighbours:
            stack.pop()
            continue

        nxc, nyc, direction = rng.choice(neighbours)
        if direction == 'W':
            vwalls.discard((cx, cy))
        elif direction == 'E':
            vwalls.discard((cx + 1, cy))
        elif direction == 'S':
            hwalls.discard((cx, cy))
        else:
            hwalls.discard((cx, cy + 1))
        visited[nxc][nyc] = True
        stack.append((nxc, nyc))

    return vwalls, hwalls


def carve_rooms(vwalls, hwalls, nx, ny, n_rooms, rng):
    """Open up 2x2 rooms (the brief's standard room size). Returns room centres."""
    centres = []
    attempts = 0
    while len(centres) < n_rooms and attempts < 200:
        attempts += 1
        i = rng.randrange(0, nx - 1)
        j = rng.randrange(0, ny - 1)
        # keep rooms apart so they stay recognisable as rooms
        if any(abs(i - ci) < 3 and abs(j - cj) < 3 for ci, cj in centres):
            continue
        # remove the 4 internal panels of the 2x2 block
        vwalls.discard((i + 1, j))
        vwalls.discard((i + 1, j + 1))
        hwalls.discard((i, j + 1))
        hwalls.discard((i + 1, j + 1))
        centres.append((i, j))
    return centres


def add_loops(vwalls, hwalls, nx, ny, fraction, rng):
    """Remove some interior panels so the maze isn't a pure tree.

    Loops matter for SLAM: they give Cartographer loop-closure opportunities,
    and they give the explorer more than one route (closer to a real building).
    """
    interior_v = [(i, j) for (i, j) in vwalls if 0 < i < nx]
    interior_h = [(i, j) for (i, j) in hwalls if 0 < j < ny]
    for wall in rng.sample(interior_v, int(len(interior_v) * fraction)):
        vwalls.discard(wall)
    for wall in rng.sample(interior_h, int(len(interior_h) * fraction)):
        hwalls.discard(wall)


def verify_connected(vwalls, hwalls, nx, ny, start):
    """Flood fill from the entry cell; every cell must be reachable."""
    seen = {start}
    stack = [start]
    while stack:
        cx, cy = stack.pop()
        moves = (
            (cx - 1, cy, (cx, cy) not in vwalls),
            (cx + 1, cy, (cx + 1, cy) not in vwalls),
            (cx, cy - 1, (cx, cy) not in hwalls),
            (cx, cy + 1, (cx, cy + 1) not in hwalls),
        )
        for tx, ty, open_ in moves:
            if open_ and 0 <= tx < nx and 0 <= ty < ny and (tx, ty) not in seen:
                seen.add((tx, ty))
                stack.append((tx, ty))
    return len(seen), nx * ny


def merge_runs(walls, vertical):
    """Merge collinear adjacent panels into single boxes (fewer SDF entities)."""
    boxes = []
    # group by the fixed axis, then walk consecutive runs along the free axis
    groups = {}
    for a, b in walls:
        key, val = (a, b) if vertical else (b, a)
        groups.setdefault(key, []).append(val)
    for key, vals in groups.items():
        for val in sorted(vals):
            if boxes and boxes[-1][0] == key and boxes[-1][2] == val:
                boxes[-1][2] = val + 1          # extend the current run
            else:
                boxes.append([key, val, val + 1])
    return boxes


def build_world(args):
    rng = random.Random(args.seed)
    cell = args.corridor
    nx = ny = int(args.size / cell)

    vwalls, hwalls = carve_maze(nx, ny, rng)
    rooms = carve_rooms(vwalls, hwalls, nx, ny, args.rooms, rng)
    add_loops(vwalls, hwalls, nx, ny, args.loops, rng)

    # Entry/exit: one gap in the south boundary. Entry and exit are the same
    # point per the brief, so the drone leaves the way it came in.
    entry_i = nx // 2
    hwalls.discard((entry_i, 0))

    reached, total = verify_connected(vwalls, hwalls, nx, ny, (entry_i, 0))
    if reached != total:
        raise SystemExit(
            f'Maze is disconnected: only {reached}/{total} cells reachable. '
            f'Try a different --seed.')

    # Offset so the entry cell centre sits at the world origin, because PX4
    # spawns the drone at (0,0) -- so it starts exactly at the entry point.
    ox = -(entry_i + 0.5) * cell
    oy = -0.5 * cell

    parts = []
    for key, lo, hi in merge_runs(vwalls, vertical=True):
        length = (hi - lo) * cell
        x = key * cell + ox
        y = (lo * cell + length / 2.0) + oy
        parts.append((x, y, WALL_THICKNESS, length))
    for key, lo, hi in merge_runs(hwalls, vertical=False):
        length = (hi - lo) * cell
        y = key * cell + oy
        x = (lo * cell + length / 2.0) + ox
        parts.append((x, y, length, WALL_THICKNESS))

    return parts, rooms, nx, ny, cell, (ox, oy), (reached, total)


HEADER = """<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <world name="{name}">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene>
      <grid>false</grid>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>1 1</size></plane></geometry>
          <surface><friction><ode/></friction><bounce/><contact/></surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
        <pose>0 0 0 0 -0 0</pose>
      </link>
      <pose>0 0 0 0 -0 0</pose>
    </model>
    <light name="sunUTC" type="directional">
      <pose>0 0 500 0 -0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>1</intensity>
      <direction>0.001 0.625 -0.78</direction>
      <diffuse>0.904 0.904 0.904 1</diffuse>
      <specular>0.271 0.271 0.271 1</specular>
      <attenuation>
        <range>2000</range><linear>0</linear><constant>1</constant><quadratic>0</quadratic>
      </attenuation>
      <spot><inner_angle>0</inner_angle><outer_angle>0</outer_angle><falloff>0</falloff></spot>
    </light>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>47.397971057728974</latitude_deg>
      <longitude_deg>8.546163739800146</longitude_deg>
      <elevation>0</elevation>
    </spherical_coordinates>
    <model name="maze">
      <static>true</static>
      <link name="walls">
"""

FOOTER = """      </link>
      <pose>0 0 0 0 0 0</pose>
    </model>
  </world>
</sdf>
"""

PANEL = """        <collision name="c{n}">
          <pose>{x:.3f} {y:.3f} {hz:.3f} 0 0 0</pose>
          <geometry><box><size>{sx:.3f} {sy:.3f} {h:.3f}</size></box></geometry>
        </collision>
        <visual name="v{n}">
          <pose>{x:.3f} {y:.3f} {hz:.3f} 0 0 0</pose>
          <geometry><box><size>{sx:.3f} {sy:.3f} {h:.3f}</size></box></geometry>
          <material>
            <ambient>0.9 0.9 0.87 1</ambient>
            <diffuse>0.9 0.9 0.87 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--size', type=float, default=14.0,
                    help='arena side in metres (brief caps the arena at 15)')
    ap.add_argument('--corridor', type=float, default=1.5,
                    help='corridor clear width; 1.0 matches the exact brief spec')
    ap.add_argument('--rooms', type=int, default=6,
                    help='number of 2x2 rooms (brief places up to 6 survivors)')
    ap.add_argument('--loops', type=float, default=0.12,
                    help='fraction of interior panels removed to create loops')
    ap.add_argument('--seed', type=int, default=42)
    ap.add_argument('-o', '--output',
                    default=os.path.expanduser(
                        '~/PX4-Autopilot/Tools/simulation/gz/worlds/nidar_maze.sdf'))
    args = ap.parse_args()

    parts, rooms, nx, ny, cell, offset, (reached, total) = build_world(args)
    name = os.path.splitext(os.path.basename(args.output))[0]

    body = ''.join(
        PANEL.format(n=i, x=x, y=y, sx=sx, sy=sy,
                     h=WALL_HEIGHT, hz=WALL_HEIGHT / 2.0)
        for i, (x, y, sx, sy) in enumerate(parts))

    with open(args.output, 'w') as fh:
        fh.write(HEADER.format(name=name) + body + FOOTER)

    print(f'Wrote {args.output}')
    print(f'  world name   : {name}   (use PX4_GZ_WORLD={name})')
    print(f'  arena        : {nx * cell:.1f}m x {ny * cell:.1f}m '
          f'({nx}x{ny} cells @ {cell}m)')
    print(f'  corridors    : {cell}m clear width')
    print(f'  wall panels  : {len(parts)} boxes, {WALL_HEIGHT}m tall, '
          f'{WALL_THICKNESS}m thick')
    print(f'  rooms        : {len(rooms)} (2x2)')
    print(f'  connectivity : {reached}/{total} cells reachable from entry  OK')
    print(f'  entry/exit   : world origin (0,0) - drone spawns at the entry')


if __name__ == '__main__':
    main()
