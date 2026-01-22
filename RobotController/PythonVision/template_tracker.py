import cv2
from tracker_base import BaseTracker, PoseResult
from config import TM_METHOD


class TemplateTracker(BaseTracker):
    name = "TEMPLATE"
    def __init__(self):
        self.template = None
        self.roi = None
        self.w = None
        self.h = None

    def init(self, frame_bgr, roi):
        self.roi = roi
        x, y, w, h = roi
        self.w, self.h = w, h
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        self.template = gray[y:y+h, x:x+w].copy()
        return True

    def update(self, frame_bgr):
        if self.template is None:
            return PoseResult(False)
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        res = cv2.matchTemplate(gray, self.template, TM_METHOD)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)

        if TM_METHOD in [cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED]:
            score = 1.0 - float(min_val)
            top_left = min_loc
        else:
            score = float(max_val)
            top_left = max_loc

        x, y = top_left
        center = (x + self.w / 2.0, y + self.h / 2.0)

        # NOTE: template matching alone doesn't give rotation reliably.
        return PoseResult(True, center=center, angle_deg=0.0, score=score, extra={"score": score})

