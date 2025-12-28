import cv2
import numpy as np
import math
from tracker_base import BaseTracker, PoseResult
from utils import roi_center
from config import ORB_NFEATURES


class ORBHomographyTracker(BaseTracker):
    name = "ORB_H"
    def __init__(self):
        self.roi = None
        self.ref_kp = None
        self.ref_desc = None
        self.ref_pts = None
        self.orb = cv2.ORB_create(nfeatures=ORB_NFEATURES)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        self.ref_center = None

    def init(self, frame_bgr, roi):
        self.roi = roi
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        x, y, w, h = roi
        patch = gray[y:y+h, x:x+w]

        kp, desc = self.orb.detectAndCompute(patch, None)
        if desc is None or len(kp) < 12:
            self.ref_kp, self.ref_desc = None, None
            return False

        # Convert keypoints to full-image coordinates
        ref_pts = np.float32([k.pt for k in kp]).reshape(-1, 1, 2)
        ref_pts[:, 0, 0] += x
        ref_pts[:, 0, 1] += y

        self.ref_kp = kp
        self.ref_desc = desc
        self.ref_pts = ref_pts
        self.ref_center = roi_center(roi)
        return True

    def update(self, frame_bgr):
        if self.ref_desc is None:
            return PoseResult(False)

        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        kp2, desc2 = self.orb.detectAndCompute(gray, None)
        if desc2 is None or len(kp2) < 12:
            return PoseResult(False)

        # KNN match + ratio test
        matches = self.bf.knnMatch(self.ref_desc, desc2, k=2)
        good = []
        for m_n in matches:
            if len(m_n) != 2:
                continue
            m, n = m_n
            if m.distance < 0.75 * n.distance:
                good.append(m)

        if len(good) < 10:
            return PoseResult(False, extra={"good_matches": len(good)})

        src = np.float32([self.ref_pts[m.queryIdx, 0] for m in good]).reshape(-1, 1, 2)
        dst = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)

        H, mask = cv2.findHomography(src, dst, cv2.RANSAC, 4.0)
        if H is None:
            return PoseResult(False, extra={"good_matches": len(good)})

        # Transform reference center
        rc = np.array([[[self.ref_center[0], self.ref_center[1]]]], dtype=np.float32)
        tc = cv2.perspectiveTransform(rc, H)[0, 0]
        center = (float(tc[0]), float(tc[1]))

        # Approx in-plane rotation from homography by looking at transformed x-axis direction
        # Take a small vector to the right of center in reference frame:
        v = np.array([[[self.ref_center[0] + 50.0, self.ref_center[1]]]], dtype=np.float32)
        tv = cv2.perspectiveTransform(v, H)[0, 0]
        dx, dy = (tv[0] - tc[0]), (tv[1] - tc[1])
        angle_deg = math.degrees(math.atan2(dy, dx))

        inliers = int(mask.sum()) if mask is not None else None
        extra = {"good_matches": len(good), "inliers": inliers}
        return PoseResult(True, center=center, angle_deg=angle_deg, extra=extra)

    def draw(self, frame_bgr, result: PoseResult, color):
        return super().draw(frame_bgr, result, color)

