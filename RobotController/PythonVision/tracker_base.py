from utils import draw_pose


class PoseResult:
    def __init__(self, ok, center=None, angle_deg=None, score=None, extra=None):
        self.ok = ok
        self.center = center
        self.angle_deg = angle_deg
        self.score = score
        self.extra = extra or {}


class BaseTracker:
    name = "Base"
    def init(self, frame_bgr, roi):
        raise NotImplementedError
    def update(self, frame_bgr):
        raise NotImplementedError
    def draw(self, frame_bgr, result: PoseResult, color):
        if result.ok and result.center is not None:
            draw_pose(frame_bgr, result.center, result.angle_deg or 0.0, color=color)
        return frame_bgr

