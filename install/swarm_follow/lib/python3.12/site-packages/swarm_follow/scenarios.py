"""
Generate reproducible starting poses for the five robots.

The same seed always gives the same poses, so every mode can be compared in
identical initial conditions. Seed 0 keeps the poses written in arena.wbt.
"""

import math
import random
import re

ROBOT_COUNT = 5
# Keep robots away from the walls of the 3 x 3 m arena and from each other.
SPAWN_HALF_SIZE = 1.2   # m
MIN_SEPARATION = 0.35   # m between robot centres


def random_poses(seed, count=ROBOT_COUNT):
    """Return [(x, y, yaw)] for count robots, always the same for a seed."""
    rng = random.Random(seed)
    poses = []
    while len(poses) < count:
        x = rng.uniform(-SPAWN_HALF_SIZE, SPAWN_HALF_SIZE)
        y = rng.uniform(-SPAWN_HALF_SIZE, SPAWN_HALF_SIZE)
        if all(math.hypot(x - px, y - py) >= MIN_SEPARATION
               for px, py, _ in poses):
            poses.append((x, y, rng.uniform(-math.pi, math.pi)))
    return poses


def world_with_poses(world_text, poses):
    """Return the world text with DEF ROBOT1..N moved to the given poses."""
    for i, (x, y, yaw) in enumerate(poses, start=1):
        pattern = (rf'(DEF ROBOT{i} E-puck \{{\s*translation )[^\n]*'
                   rf'(\n\s*rotation )[^\n]*')
        world_text, found = re.subn(
            pattern,
            rf'\g<1>{x:.6f} {y:.6f} 0\g<2>0 0 1 {yaw:.9f}',
            world_text)
        if not found:
            raise ValueError(f'DEF ROBOT{i} not found in world')
    return world_text
