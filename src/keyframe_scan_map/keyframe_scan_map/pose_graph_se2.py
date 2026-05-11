"""Planar SE(2) helpers for lightweight pose-graph residuals (numpy 3x3)."""
from __future__ import annotations

import math
import numpy as np
import numpy.linalg as la


def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def T_from_xyw(p: np.ndarray) -> np.ndarray:
    x, y, w = float(p[0]), float(p[1]), float(p[2])
    c, s = math.cos(w), math.sin(w)
    t = np.eye(3, dtype=np.float64)
    t[0, 0] = c
    t[0, 1] = -s
    t[0, 2] = x
    t[1, 0] = s
    t[1, 1] = c
    t[1, 2] = y
    return t


def odom_measurement(P: np.ndarray, i: int, j: int) -> np.ndarray:
    """Relative transform T_i^{-1} T_j from current poses (3x3)."""
    return la.inv(T_from_xyw(P[i])) @ T_from_xyw(P[j])


def residual_between(pred_t: np.ndarray, meas_t: np.ndarray) -> np.ndarray:
    """Log-like error vector (3,) from pred and meas SE2 homogeneous (3x3)."""
    e = la.inv(meas_t) @ pred_t
    return np.array(
        [float(e[0, 2]), float(e[1, 2]), wrap_angle(math.atan2(float(e[1, 0]), float(e[0, 0])))],
        dtype=np.float64,
    )


def transform_points_se2(T: np.ndarray, pts: np.ndarray) -> np.ndarray:
    """Apply SE(2) in xy; z column unchanged (planar correction of map points)."""
    if pts.size == 0:
        return pts
    n = pts.shape[0]
    xy_h = np.hstack([pts[:, :2].astype(np.float64), np.ones((n, 1))])
    xy2 = (T @ xy_h.T).T
    return np.column_stack([xy2[:, 0], xy2[:, 1], pts[:, 2].astype(np.float32)])
