import cv2
import numpy as np
import math
from tracker_base import BaseTracker, PoseResult
from frame_diff_mask import FrameDiffMask
from utils import roi_center
from config import (
    LK_MAX_CORNERS,
    LK_QUALITY_LEVEL,
    LK_MIN_DISTANCE,
    LK_WIN_SIZE,
    LK_MAX_LEVEL,
    LK_NUM_PAIRS,
    MOVEMENT_CHECK_RADIUS,
    MOVEMENT_CHECK_THRESHOLD,
    DRAW_FEATURES,
)


class LKFlowTracker(BaseTracker):
    name = "LK_FLOW"

    def __init__(self):
        self.roi = None
        self.prev_gray = None
        self.prev_pts = None
        self.prev_center = None
        self.angle_deg = 0.0
        self.point_pairs = None  # List of (idx1, idx2) tuples for random point pairs
        self.frame_diff_mask = FrameDiffMask()

    def init(self, frame_bgr, roi):
        self.roi = roi
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        x, y, w, h = roi
        roi_gray = gray[y : y + h, x : x + w]
        pts = cv2.goodFeaturesToTrack(
            roi_gray,
            maxCorners=LK_MAX_CORNERS,
            qualityLevel=LK_QUALITY_LEVEL,
            minDistance=LK_MIN_DISTANCE,
        )
        if pts is None or len(pts) < 8:
            self.prev_pts = None
            self.prev_gray = gray
            self.prev_center = roi_center(roi)
            self.angle_deg = 0.0
            self.point_pairs = None
            return False

        # shift to full-image coords
        pts[:, 0, 0] += x
        pts[:, 0, 1] += y

        self.prev_pts = pts.astype(np.float32)
        self.prev_gray = gray
        self.prev_center = roi_center(roi)
        self.angle_deg = 0.0

        self.point_pairs = self._initialize_point_pairs(len(pts))

        return True

    def _initialize_point_pairs(self, n_pts: int) -> list[tuple[int, int]]:
        """Returns a list of (idx1, idx2) tuples for random point pairs."""
        if n_pts < 2:
            return []

        num_pairs = min(LK_NUM_PAIRS, n_pts * (n_pts - 1) // 2)
        all_pairs = [(i, j) for i in range(n_pts) for j in range(i + 1, n_pts)]

        # if there are more pairs than we need, select a random subset
        if len(all_pairs) > num_pairs:
            selected_indices = np.random.choice(
                len(all_pairs), num_pairs, replace=False
            )
            return [all_pairs[idx] for idx in selected_indices]
        else:
            return all_pairs

    def _angle_wrap(self, angle_deg: float) -> float:
        """Normalize angle to [-180, 180] range."""
        while angle_deg > 180:
            angle_deg -= 360
        while angle_deg < -180:
            angle_deg += 360
        return angle_deg

    def _compute_center_from_points(self, points):
        """Compute center (centroid) from a set of points."""
        if len(points) == 0:
            return None
        return (float(points[:, 0].mean()), float(points[:, 1].mean()))

    def _apply_mask_filtering(self, next_pts, center, gray):
        """Apply frame diff mask filtering to points if movement is detected."""
        self.frame_diff_mask.compute_mask(self.prev_gray, gray)
        if self.frame_diff_mask.current_mask is None:
            return next_pts, center, False

        has_movement = self.frame_diff_mask.check_movement_near_center(
            center, MOVEMENT_CHECK_RADIUS, MOVEMENT_CHECK_THRESHOLD
        )

        if not has_movement:
            return next_pts, center, False

        # Filter points to only keep those in the mask
        keep_indices = self.frame_diff_mask.get_indices_in_mask(next_pts)

        if len(keep_indices) < 6:
            return next_pts, center, False

        # Filter both prev_pts and next_pts to keep indices aligned
        filtered_next_pts = next_pts[keep_indices]
        self.prev_pts = self.prev_pts[keep_indices]

        # Recompute center from filtered points
        filtered_next_pts_flat = filtered_next_pts[:, 0]
        new_center = self._compute_center_from_points(filtered_next_pts_flat)

        # Rebuild point_pairs from filtered set
        self.point_pairs: list[tuple[int, int]] = self._initialize_point_pairs(
            len(filtered_next_pts)
        )

        return filtered_next_pts, new_center, True

    def _compute_rotations_from_pairs(self, next_pts: np.ndarray, status: np.ndarray) -> list[float]:
        """
        Compute rotation angles from point pairs.
        Returns a list of rotation angles in degrees.
        next_pts: (N, 2) array of next frame points
        status: (N,) mask that is 1 for good points and 0 for bad points
        """
        rotations: list[float] = []
        if self.point_pairs is None or len(self.point_pairs) == 0:
            return rotations

        n_pts = len(self.prev_pts)
        for idx1, idx2 in self.point_pairs:
            if not (
                idx1 < n_pts
                and idx2 < n_pts
                and idx1 < len(status)
                and idx2 < len(status)
                and status[idx1] == 1
                and status[idx2] == 1
            ):
                continue

            # Get points in prev and next frames
            p1_prev = self.prev_pts[idx1, 0]
            p2_prev = self.prev_pts[idx2, 0]
            p1_next = next_pts[idx1, 0]
            p2_next = next_pts[idx2, 0]

            # Compute vectors between points
            vec_prev = p2_prev - p1_prev
            vec_next = p2_next - p1_next

            # Skip if vectors are too short (unreliable angle)
            if np.linalg.norm(vec_prev) < 3.0 or np.linalg.norm(vec_next) < 3.0:
                continue

            # Compute angles
            angle_prev = math.degrees(math.atan2(vec_prev[1], vec_prev[0]))
            angle_next = math.degrees(math.atan2(vec_next[1], vec_next[0]))
            rot = self._angle_wrap(angle_next - angle_prev)

            rotations.append(rot)

        return rotations

    def _update_angle_from_rotations(self, rotations: list[float]) -> float:
        """Update angle_deg from list of rotation deltas."""
        if len(rotations) >= 3:
            rotations_arr = np.array(rotations)
            angle_delta = float(np.median(rotations_arr))
            # Accumulate rotation relative to initial orientation
            self.angle_deg += angle_delta
            self.angle_deg = self._angle_wrap(self.angle_deg)
            return angle_delta
        else:
            # Not enough pairs, keep last angle (don't update)
            return 0.0

    def update(self, frame_bgr):
        if self.prev_gray is None or self.prev_pts is None or len(self.prev_pts) < 6:
            return PoseResult(False)

        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        next_pts, st, err = cv2.calcOpticalFlowPyrLK(
            self.prev_gray,
            gray,
            self.prev_pts,
            None,
            winSize=LK_WIN_SIZE,
            maxLevel=LK_MAX_LEVEL,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
        )
        if next_pts is None:
            return PoseResult(False)

        # Get status array and filter good points
        status = st.reshape(-1)
        good_mask = status == 1
        good_indices = np.where(good_mask)[0]

        if len(good_indices) < 6:
            return PoseResult(False)

        # Compute center from tracked points (centroid of good points)
        good_next_pts = next_pts[good_mask, 0]  # Shape: (N, 2)
        center = self._compute_center_from_points(good_next_pts)

        # Apply frame diff mask filtering if movement is detected
        next_pts, center, was_filtered = self._apply_mask_filtering(
            next_pts, center, gray
        )

        # Rebuild status array if points were filtered
        if was_filtered:
            status = np.ones(len(next_pts), dtype=np.uint8)
            good_mask = status == 1
            good_indices = np.where(good_mask)[0]

        # Compute rotation using angle changes between point pairs
        rotations = self._compute_rotations_from_pairs(next_pts, status)

        # Update angle from computed rotations
        angle_delta = self._update_angle_from_rotations(rotations)

        self.prev_gray = gray
        self.prev_pts = next_pts  # Keep all points with same structure
        self.prev_center = center

        extra = {
            "num_pts": int(len(good_indices)),
            "num_pairs_used": len(rotations),
            "rotation_delta": float(angle_delta),
        }
        return PoseResult(True, center=center, angle_deg=self.angle_deg, extra=extra)

    def draw(self, frame_bgr, result: PoseResult, color):
        if result.ok and DRAW_FEATURES and self.prev_pts is not None:
            for p in self.prev_pts.reshape(-1, 2):
                cv2.circle(frame_bgr, (int(p[0]), int(p[1])), 2, color, -1, cv2.LINE_AA)
        return super().draw(frame_bgr, result, color)
