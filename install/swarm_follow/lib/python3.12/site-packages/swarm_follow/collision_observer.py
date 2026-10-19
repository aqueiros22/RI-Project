"""Read physical contacts from a separate, passive Webots Supervisor."""

import csv
import json
import math
from itertools import combinations
from pathlib import Path


def contact_pairs(points, tolerance=1e-6):
    """Find pairs sharing an actual contact point, once per pair and step."""
    pairs = []
    for first, second in combinations(sorted(points), 2):
        if not (first.startswith('robot') or second.startswith('robot')):
            continue
        if any(math.dist(a, b) <= tolerance
               for a in points[first] for b in points[second]):
            pairs.append((first, second))
    return pairs


class CollisionObserver:
    """Record physical contacts without changing the simulation or robots."""

    def init(self, webots_node, properties):
        """Resolve robot and arena nodes in the evaluation world."""
        self.robot = webots_node.robot
        self.dt = self.robot.getBasicTimeStep() / 1000.0
        self.nodes = {f'robot{i}': self.robot.getFromDef(f'ROBOT{i}')
                      for i in range(1, 6)}
        self.arena = self.robot.getFromDef('ARENA')
        if self.arena is None or any(n is None for n in self.nodes.values()):
            raise ValueError('Metrics require ARENA and ROBOT1..ROBOT5 DEFs')
        path = Path(properties['output_dir']) / 'contacts.csv'
        self.file = path.open('x', newline='', buffering=1)
        self.writer = csv.writer(self.file)
        self.writer.writerow(['time_s', 'dt_s', 'pairs_json'])

    def step(self):
        """Match contact coordinates; node_id identifies the contacted part."""
        points = {
            name: [tuple(c.point) for c in node.getContactPoints(True)]
            for name, node in self.nodes.items()
        }
        # R2025a contact.node_id is the queried solid or its descendant,
        # NOT the opposing collider. Match both objects' contact positions.
        # RectangleArena gives its four wall solids names wall, wall#2, ...
        # Its floor contacts are deliberately excluded.
        for contact in self.arena.getContactPoints(True):
            node = self.robot.getFromId(contact.node_id)
            field = node.getField('name') if node is not None else None
            name = field.getSFString() if field is not None else ''
            if name == 'wall' or name.startswith('wall#'):
                points.setdefault(name, []).append(tuple(contact.point))
        self.writer.writerow([
            self.robot.getTime(), self.dt, json.dumps(contact_pairs(points)),
        ])

    def stop(self):
        """Close the line-buffered recording on a normal shutdown."""
        self.file.close()
