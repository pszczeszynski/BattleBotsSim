import cv2
from dataclasses import dataclass


@dataclass
class ROISelector:
    selecting: bool = False
    start_pt: tuple = None
    end_pt: tuple = None
    roi: tuple = None
    done: bool = False


def make_roi_selector():
    sel = ROISelector()

    def on_mouse(event, x, y, flags, param):
        nonlocal sel
        if event == cv2.EVENT_LBUTTONDOWN:
            sel.selecting = True
            sel.start_pt = (x, y)
            sel.end_pt = (x, y)
            sel.done = False
        elif event == cv2.EVENT_MOUSEMOVE and sel.selecting:
            sel.end_pt = (x, y)
        elif event == cv2.EVENT_LBUTTONUP:
            sel.selecting = False
            sel.end_pt = (x, y)
            x0, y0 = sel.start_pt
            x1, y1 = sel.end_pt
            x_min, x_max = sorted([x0, x1])
            y_min, y_max = sorted([y0, y1])
            w = x_max - x_min
            h = y_max - y_min
            if w >= 5 and h >= 5:
                sel.roi = (x_min, y_min, w, h)
                sel.done = True

    return sel, on_mouse

