import cv2
import numpy as np
from config import FRAME_DIFF_THRESHOLD, FRAME_DIFF_BLUR_SIZE, FRAME_DIFF_RE_THRESHOLD


class FrameDiffMask:
    """Handles frame difference mask computation with blur and re-thresholding for noise rejection."""
    
    def __init__(self, diff_threshold=FRAME_DIFF_THRESHOLD, blur_size=FRAME_DIFF_BLUR_SIZE, 
                 re_threshold=FRAME_DIFF_RE_THRESHOLD):
        self.diff_threshold = diff_threshold
        self.blur_size = blur_size if blur_size % 2 == 1 else blur_size + 1  # Ensure odd
        self.re_threshold = re_threshold
        self.current_mask = None
    
    def compute_mask(self, prev_gray, curr_gray):
        """Compute pixel-wise diff mask between two frames with blur and re-thresholding."""
        if prev_gray is None or curr_gray is None:
            self.current_mask = None
            return None
        if prev_gray.shape != curr_gray.shape:
            self.current_mask = None
            return None
        
        # Compute initial diff
        diff = cv2.absdiff(prev_gray, curr_gray)
        _, mask = cv2.threshold(diff, self.diff_threshold, 255, cv2.THRESH_BINARY)
        
        # Blur the mask to reduce noise
        mask_blurred = cv2.GaussianBlur(mask, (self.blur_size, self.blur_size), 0)
        
        # Re-threshold the blurred mask (more lenient threshold)
        _, mask_final = cv2.threshold(mask_blurred, self.re_threshold, 255, cv2.THRESH_BINARY)
        
        self.current_mask = mask_final
        return mask_final
    
    def check_movement_near_center(self, center: tuple[float, float], radius: float, threshold_ratio: float) -> bool:
        """Check if there's sufficient movement near the center."""
        mask = self.current_mask
        if mask is None or center is None:
            return False
        
        cx, cy = int(center[0]), int(center[1])
        h, w = mask.shape
        
        # Create a circular mask around center
        y, x = np.ogrid[:h, :w]
        dist_sq = (x - cx) ** 2 + (y - cy) ** 2
        circle_mask = dist_sq <= radius ** 2
        
        # Count movement pixels within the circle
        movement_in_circle = np.sum((mask > 0) & circle_mask)
        total_in_circle = np.sum(circle_mask)
        
        if total_in_circle == 0:
            return False
        
        movement_ratio = movement_in_circle / total_in_circle
        return movement_ratio >= threshold_ratio
    
    def get_indices_in_mask(self, points):
        """Get indices of points that are in the mask (where mask > 0)."""
        mask = self.current_mask
        if mask is None or points is None:
            return []
        if len(points) == 0:
            return []
        
        # Points shape: (N, 1, 2)
        keep_indices = []
        for i in range(len(points)):
            x, y = int(points[i, 0, 0]), int(points[i, 0, 1])
            h, w = mask.shape
            if 0 <= y < h and 0 <= x < w and mask[y, x] > 0:
                keep_indices.append(i)
        
        return keep_indices
    
    def compute_mask_center_near_point(self, center, radius):
        """Compute the centroid of the diff mask near a given center point."""
        mask = self.current_mask
        if mask is None or center is None:
            return None
        
        cx, cy = int(center[0]), int(center[1])
        h, w = mask.shape
        
        # Create a circular mask around center
        y, x = np.ogrid[:h, :w]
        dist_sq = (x - cx) ** 2 + (y - cy) ** 2
        circle_mask = dist_sq <= radius ** 2
        
        # Find mask pixels within the circle
        mask_in_circle = (mask > 0) & circle_mask
        
        if not np.any(mask_in_circle):
            return None
        
        # Compute centroid of mask pixels within the circle
        y_coords, x_coords = np.where(mask_in_circle)
        if len(x_coords) == 0:
            return None
        
        mask_center_x = float(np.mean(x_coords))
        mask_center_y = float(np.mean(y_coords))
        return (mask_center_x, mask_center_y)

