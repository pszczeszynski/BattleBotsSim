import cv2
import numpy as np
from config import (
    VIDEO_PATH,
    START_ROI,
    ENABLE_LK_FLOW,
    ENABLE_TEMPLATE,
    ENABLE_ORB_H,
    SHOW_DEBUG_TEXT,
    VIDEO_START_TIME_S,
    CONVERT_TO_GRAY,
)
from lk_flow_tracker import LKFlowTracker
from template_tracker import TemplateTracker
from orb_tracker import ORBHomographyTracker
from roi_selector import make_roi_selector
from utils import clamp_roi, put_lines
from video_reader import VideoReader


def main():
    # Open video reader
    try:
        reader = VideoReader(VIDEO_PATH, skip_duplicates=True)
    except ValueError as e:
        print(f"Error: {e}")
        return

    # Seek to the start time
    start_frame = int(VIDEO_START_TIME_S * reader.fps)
    reader.set_frame_number(start_frame)
    
    # Get first frame for ROI selection
    ret, frame = reader.read()
    if not ret:
        print("Error: Could not read first frame")
        return

    # Initialize trackers
    trackers = {
        "LK_FLOW": (LKFlowTracker(), ENABLE_LK_FLOW, (0, 255, 0)),  # Green
        "TEMPLATE": (TemplateTracker(), ENABLE_TEMPLATE, (255, 0, 0)),  # Blue
        "ORB_H": (ORBHomographyTracker(), ENABLE_ORB_H, (0, 0, 255)),  # Red
    }

    # ROI selection
    roi = START_ROI
    if roi is None:
        print("Select ROI with mouse, then press SPACE to continue")
        roi_sel, on_mouse = make_roi_selector()
        cv2.namedWindow("Select ROI", cv2.WINDOW_NORMAL)
        cv2.setMouseCallback("Select ROI", on_mouse)

        while True:
            frame_copy = frame.copy()
            if roi_sel.selecting:
                x0, y0 = roi_sel.start_pt
                x1, y1 = roi_sel.end_pt
                cv2.rectangle(frame_copy, (x0, y0), (x1, y1), (0, 255, 0), 2)
            elif roi_sel.roi is not None:
                x, y, w, h = roi_sel.roi
                cv2.rectangle(frame_copy, (x, y), (x + w, y + h), (0, 255, 0), 2)

            cv2.putText(
                frame_copy,
                "Select ROI, then press SPACE",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )
            cv2.imshow("Select ROI", frame_copy)

            key = cv2.waitKey(1) & 0xFF
            if key == ord(" ") and roi_sel.done:
                roi = roi_sel.roi
                break
            elif key == ord("q"):
                return

        cv2.destroyWindow("Select ROI")

    # Clamp ROI to frame bounds
    roi = clamp_roi(roi, frame.shape)

    # Initialize all trackers
    print("Initializing trackers...")
    for name, (tracker, enabled, _) in trackers.items():
        if enabled:
            success = tracker.init(frame, roi)
            if success:
                print(f"  {name}: OK")
            else:
                print(f"  {name}: FAILED")
                trackers[name] = (tracker, False, trackers[name][2])

    # Main loop
    print("\nControls:")
    print("  1 - Toggle LK Flow tracker")
    print("  2 - Toggle Template tracker")
    print("  3 - Toggle ORB Homography tracker")
    print("  SPACE - Pause/Resume")
    print("  N - Step forward one frame (when paused)")
    print("  P - Step backward one frame (when paused)")
    print("  Q - Quit")
    print("\nPress any key to start...")

    # Create resizable main window
    cv2.namedWindow("Tracker", cv2.WINDOW_NORMAL)
    # Create resizable window for diff mask
    cv2.namedWindow("Frame Diff Mask", cv2.WINDOW_NORMAL)

    paused = False
    step_forward = False
    step_backward = False

    while True:
        if not paused or step_forward or step_backward:
            if step_backward:
                # Step backward: go to previous frame
                current_frame = reader.get_frame_number()
                if current_frame > 0:
                    reader.set_frame_number(current_frame - 1)
                step_backward = False
            
            ret, frame = reader.read()

            if not ret:
                if step_forward or step_backward:
                    # If stepping and at end, don't loop automatically
                    step_forward = False
                    step_backward = False
                    continue
                # Loop video (only when not paused/stepping)
                reader.set_frame_number(0)
                ret, frame = reader.read()
                if not ret:
                    break
            
            step_forward = False

        # Convert to grayscale if enabled
        if CONVERT_TO_GRAY:
            if len(frame.shape) == 3:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                # Convert back to BGR for display (so colors work)
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

        frame_display = frame.copy()
        # Update and draw enabled trackers
        debug_lines = []
        for name, (tracker, enabled, color) in trackers.items():
            if not enabled:
                continue

            result = tracker.update(frame)
            if result.ok:
                frame_display = tracker.draw(frame_display, result, color)

                # Add debug info
                if SHOW_DEBUG_TEXT:
                    info = f"{name}: OK"
                    if result.center:
                        info += f" @ ({int(result.center[0])}, {int(result.center[1])})"
                    if result.angle_deg is not None:
                        info += f" angle={result.angle_deg:.1f}°"
                    if result.extra:
                        extra_strs = [f"{k}={v}" for k, v in result.extra.items()]
                        info += " " + " ".join(extra_strs)
                    debug_lines.append(info)
            else:
                if SHOW_DEBUG_TEXT:
                    debug_lines.append(f"{name}: FAILED")

        # Draw debug text
        if SHOW_DEBUG_TEXT:
            current_frame_pos = reader.get_frame_number()
            debug_lines.insert(0, f"Frame: {current_frame_pos}")
            if paused:
                debug_lines.insert(1, "PAUSED")
            put_lines(frame_display, debug_lines)

        # Show frame
        cv2.imshow("Tracker", frame_display)

        # Show frame diff mask if LK tracker is enabled
        lk_tracker, lk_enabled, _ = trackers["LK_FLOW"]
        if lk_enabled and hasattr(lk_tracker, 'frame_diff_mask'):
            mask = lk_tracker.frame_diff_mask.current_mask
            if mask is not None:
                # Convert grayscale mask to BGR for display
                mask_display = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
                cv2.imshow("Frame Diff Mask", mask_display)
            else:
                # Show black frame if mask is not available
                h, w = frame.shape[:2]
                black_frame = np.zeros((h, w, 3), dtype=np.uint8)
                cv2.imshow("Frame Diff Mask", black_frame)

        # Handle keys
        key = cv2.waitKey(1 if not paused else 0) & 0xFF
        if key == ord("q"):
            break
        elif key == ord(" "):
            paused = not paused
        elif key == ord("n") or key == ord("N"):
            # Step forward one frame
            if paused:
                step_forward = True
        elif key == ord("p") or key == ord("P"):
            # Step backward one frame
            if paused:
                step_backward = True
        elif key == ord("1"):
            tracker, enabled, color = trackers["LK_FLOW"]
            trackers["LK_FLOW"] = (tracker, not enabled, color)
            if not enabled:
                # Re-initialize if enabling
                tracker.init(frame, roi)
        elif key == ord("2"):
            tracker, enabled, color = trackers["TEMPLATE"]
            trackers["TEMPLATE"] = (tracker, not enabled, color)
            if not enabled:
                tracker.init(frame, roi)
        elif key == ord("3"):
            tracker, enabled, color = trackers["ORB_H"]
            trackers["ORB_H"] = (tracker, not enabled, color)
            if not enabled:
                tracker.init(frame, roi)

    reader.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

