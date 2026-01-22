import cv2
import numpy as np
import math


def kabsch_angle_deg(X, Y):
    # X,Y: (N,2) centered point sets
    H = X.T @ Y
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[1, :] *= -1
        R = Vt.T @ U.T
    return math.degrees(math.atan2(R[1,0], R[0,0]))


def clamp_roi(roi, shape):
    x, y, w, h = roi
    H, W = shape[:2]
    x = max(0, min(x, W - 1))
    y = max(0, min(y, H - 1))
    w = max(1, min(w, W - x))
    h = max(1, min(h, H - y))
    return (x, y, w, h)


def roi_center(roi):
    x, y, w, h = roi
    return (x + w / 2.0, y + h / 2.0)


def draw_pose(img, center, angle_deg, color=(255, 0, 255), axis_len=120):
    """Draw heading (angle_deg) as a crosshair centered at center."""
    cx, cy = center
    ang = math.radians(angle_deg)

    # Heading line
    x2 = cx + axis_len * math.cos(ang)
    y2 = cy + axis_len * math.sin(ang)
    x1 = cx - axis_len * 0.6 * math.cos(ang)
    y1 = cy - axis_len * 0.6 * math.sin(ang)
    cv2.line(img, (int(x1), int(y1)), (int(x2), int(y2)), color, 4, cv2.LINE_AA)

    # Perpendicular crosshair line
    angp = ang + math.pi / 2.0
    x3 = cx + axis_len * 0.45 * math.cos(angp)
    y3 = cy + axis_len * 0.45 * math.sin(angp)
    x4 = cx - axis_len * 0.45 * math.cos(angp)
    y4 = cy - axis_len * 0.45 * math.sin(angp)
    cv2.line(img, (int(x4), int(y4)), (int(x3), int(y3)), color, 4, cv2.LINE_AA)

    cv2.circle(img, (int(cx), int(cy)), 4, color, -1, cv2.LINE_AA)


def median_angle_deg(angles_deg):
    """Median of angles in degrees, with wrap-around handling."""
    if len(angles_deg) == 0:
        return None
    # Convert to unit vectors then average direction robustly via median on atan2 is messy.
    # Use circular mean-ish fallback:
    ang = np.deg2rad(np.array(angles_deg, dtype=np.float32))
    s = np.sin(ang).mean()
    c = np.cos(ang).mean()
    return float(np.rad2deg(np.arctan2(s, c)))


def put_lines(img, lines, org=(10, 25), dy=22, color=(255, 255, 255)):
    x, y = org
    for i, t in enumerate(lines):
        cv2.putText(img, t, (x, y + i * dy), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 3, cv2.LINE_AA)
        cv2.putText(img, t, (x, y + i * dy), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1, cv2.LINE_AA)

