"""
Accumulate the assignment metrics from simulator poses and robot reports.

Poses come from the Webots supervisor and are used only for evaluation; they
are never sent back to the controllers.
"""

import math

EPUCK_RADIUS = 0.037      # m, body cylinder of the e-puck PROTO.
ARENA_HALF_SIZE = 1.5     # m, the RectangleArena is 3 x 3 m.
# Contact when centres are within two radii (plus ~1 mm of contact softness).
ROBOT_CONTACT_DISTANCE = 0.075
WALL_CONTACT_LIMIT = ARENA_HALF_SIZE - EPUCK_RADIUS - 0.002
# A contact only ends after separating this much further (stops chattering).
CONTACT_HYSTERESIS = 0.005
# Detection gaps up to this long are vision flicker, not a lost target.
MIN_LOSS_DURATION = 0.2   # s

SUMMARY_FIELDS = [
    'mode', 'seed', 'robot', 'duration_s', 'following_time_s', 'following_pct',
    'mean_abs_bearing_deg', 'target_losses', 'robot_collisions',
    'wall_collisions',
]


class RobotStats:
    """Running totals for one robot."""

    def __init__(self):
        self.following_time = 0.0
        self.bearing_sum = 0.0
        self.bearing_samples = 0
        self.losses = 0
        self.robot_collisions = 0
        self.wall_collisions = 0
        # Debounced following state and how long the current gap has lasted.
        self.tracking = False
        self.gap = 0.0
        self.touching_wall = False


class MetricsTracker:
    """Update the metrics once per simulation step."""

    def __init__(self, names, dt):
        self.names = list(names)
        self.dt = dt
        self.duration = 0.0
        self.stats = {name: RobotStats() for name in self.names}
        self.pair_contacts = 0
        self.touching = set()

    def update(self, poses, states):
        """
        Add one step.

        poses: {name: (x, y, yaw)} from the simulator.
        states: {name: (following, bearing)} as last reported by each robot,
        or None while a robot has not reported yet.
        """
        self.duration += self.dt
        for name in self.names:
            self._update_following(self.stats[name], states.get(name))
            self._update_wall(self.stats[name], poses[name])
        self._update_robot_contacts(poses)

    def _update_following(self, stats, state):
        following, bearing = state if state is not None else (False, math.nan)
        if following:
            stats.following_time += self.dt
            if not math.isnan(bearing):
                stats.bearing_sum += abs(bearing)
                stats.bearing_samples += 1
            stats.tracking, stats.gap = True, 0.0
        elif stats.tracking:
            stats.gap += self.dt
            if stats.gap > MIN_LOSS_DURATION:
                stats.losses += 1
                stats.tracking = False

    def _update_wall(self, stats, pose):
        reach = max(abs(pose[0]), abs(pose[1]))
        if not stats.touching_wall and reach > WALL_CONTACT_LIMIT:
            stats.wall_collisions += 1
            stats.touching_wall = True
        elif reach < WALL_CONTACT_LIMIT - CONTACT_HYSTERESIS:
            stats.touching_wall = False

    def _update_robot_contacts(self, poses):
        for i, a in enumerate(self.names):
            for b in self.names[i + 1:]:
                distance = math.hypot(poses[a][0] - poses[b][0],
                                      poses[a][1] - poses[b][1])
                pair = (a, b)
                if pair not in self.touching and (
                        distance < ROBOT_CONTACT_DISTANCE):
                    self.touching.add(pair)
                    self.pair_contacts += 1
                    self.stats[a].robot_collisions += 1
                    self.stats[b].robot_collisions += 1
                elif distance > ROBOT_CONTACT_DISTANCE + CONTACT_HYSTERESIS:
                    self.touching.discard(pair)

    def summary_rows(self, mode, seed=0):
        """Return one row per robot and an 'all' row for the swarm."""
        rows = []
        for name in self.names:
            s = self.stats[name]
            rows.append(self._row(
                mode, seed, name, s.following_time, s.bearing_sum,
                s.bearing_samples, s.losses, s.robot_collisions,
                s.wall_collisions))
        stats = self.stats.values()
        count = max(1, len(self.names))
        rows.append(self._row(
            mode, seed, 'all',
            sum(s.following_time for s in stats) / count,
            sum(s.bearing_sum for s in stats),
            sum(s.bearing_samples for s in stats),
            sum(s.losses for s in stats),
            self.pair_contacts,
            sum(s.wall_collisions for s in stats)))
        return rows

    def _row(self, mode, seed, robot, following_time, bearing_sum, samples,
             losses, robot_collisions, wall_collisions):
        duration = self.duration
        return {
            'mode': mode,
            'seed': seed,
            'robot': robot,
            'duration_s': round(duration, 3),
            'following_time_s': round(following_time, 3),
            'following_pct': round(
                100 * following_time / duration, 2) if duration else 0.0,
            'mean_abs_bearing_deg': round(
                math.degrees(bearing_sum / samples), 2) if samples else '',
            'target_losses': losses,
            'robot_collisions': robot_collisions,
            'wall_collisions': wall_collisions,
        }
