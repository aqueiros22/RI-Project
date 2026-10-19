"""Detect the unmodified e-puck v1 appearance in a level frontal RGB image."""

from dataclasses import dataclass
import math

import cv2
import numpy as np


@dataclass(frozen=True)
class Observation:
    """Describe a visual target without its identity or world position."""

    bearing: float
    angular_height: float
    clipped: bool


def detect_robot(bgra, field_of_view):
    """
    Find a green circuit board supported by the e-puck's grey body.

    This scene-specific heuristic assumes stock v1 robots on a flat floor
    and a level camera at body height. It uses only pixels and camera FOV.
    Apparent size is the board's elevation above the image horizon, corrected
    for bearing. It is a visual control signal, not a measured distance.
    """
    height, width = bgra.shape[:2]
    horizon = (height - 1) / 2
    focal = width / (2 * math.tan(field_of_view / 2))
    bgr = np.ascontiguousarray(bgra[:, :, :3])
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

    # Existing green PCB; no painted target, recognition mask or object IDs.
    board = cv2.inRange(hsv, (35, 65, 25), (85, 255, 255))
    board[:int(height * 0.1)] = 0
    board[int(horizon - 3):] = 0
    joined = cv2.morphologyEx(
        board, cv2.MORPH_CLOSE, np.ones((1, 3), dtype=np.uint8))
    _, _, components, _ = cv2.connectedComponentsWithStats(joined)
    green_y, green_x = np.nonzero(board)

    blue, _, red = cv2.split(bgr.astype(np.float32))
    grey = ((hsv[:, :, 1] <= 100) & (hsv[:, :, 2] >= 25) &
            (hsv[:, :, 2] <= 230) & (blue >= 0.85 * red))
    best = None
    for x, y, w, h, area in components[1:]:
        if area < 2:
            continue
        elevation = horizon - y
        # Join fragments of the same PCB, including its shaded centre.
        nearby = ((np.abs(green_x - (x + (w - 1) / 2)) <=
                   3 * elevation) &
                  (np.abs(green_y - y) <= max(2, elevation * 0.3)))
        xs, ys = green_x[nearby], green_y[nearby]
        if len(xs) < 4:
            continue
        left, right = int(xs.min()), int(xs.max())
        top, bottom = int(ys.min()), int(ys.max())
        span = right - left + 1
        if span < 5 or span < 1.5 * (bottom - top + 1):
            continue
        center = (left + right) / 2
        elevation = horizon - top
        # Look for the neutral body below the board, across the horizon.
        radius = max(span / 2, elevation * 1.2)
        x0 = max(0, int(center - radius))
        x1 = min(width, int(center + radius + 1))
        y0 = max(0, int(horizon - elevation * 0.35))
        y1 = min(height, int(horizon + elevation))
        body = grey[y0:y1, x0:x1]
        if body.size == 0 or np.mean(body) < 0.20:
            continue
        bearing = math.atan2((width - 1) / 2 - center, focal)
        angular_height = math.atan2(elevation * math.cos(bearing), focal)
        candidate = Observation(
            bearing, angular_height, left == 0 or right == width - 1)
        # Identical robots: larger apparent elevation usually means nearer.
        if best is None or candidate.angular_height > best.angular_height:
            best = candidate
    return best
