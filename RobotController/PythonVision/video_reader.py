import cv2
import numpy as np


class VideoReader:
    """Video reader that automatically skips duplicate frames by probing a few pixels."""
    
    def __init__(self, video_path, skip_duplicates=True, num_probes=100, tolerance=10.0):
        """
        Initialize video reader.
        
        Args:
            video_path: Path to video file
            skip_duplicates: If True, automatically skip duplicate frames
            num_probes: Number of random pixel locations to probe for duplicate detection
            tolerance: Pixel value tolerance for considering frames identical (default: 1.0)
        """
        self.cap = cv2.VideoCapture(video_path)
        if not self.cap.isOpened():
            raise ValueError(f"Could not open video {video_path}")
        
        self.skip_duplicates = skip_duplicates
        self.num_probes = num_probes
        self.tolerance = tolerance
        self.prev_frame = None
        self.probe_locations = None
        self.current_frame_num = 0  # Track current frame number to avoid queries
        
        # Get video properties
        self.fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.total_frames = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    
    def _initialize_probes(self, frame):
        """Initialize random probe locations for duplicate detection."""
        h, w = frame.shape[:2]
        # Generate random probe locations
        self.probe_locations = [
            (np.random.randint(0, w), np.random.randint(0, h))
            for _ in range(self.num_probes)
        ]
    
    def _frames_identical(self, frame1, frame2):
        """Check if two frames are identical by probing a few pixel locations."""
        if frame1 is None or frame2 is None:
            return False
        
        if frame1.shape != frame2.shape:
            return False
        
        # Initialize probes on first comparison
        if self.probe_locations is None:
            self._initialize_probes(frame1)
        
        # Check if probes match within tolerance
        for x, y in self.probe_locations:
            pixel1 = frame1[y, x].astype(np.float32)
            pixel2 = frame2[y, x].astype(np.float32)
            if not np.allclose(pixel1, pixel2, atol=self.tolerance):
                print("different pixels")
                return False
        print("same pixels")
        return True
    
    def _seek_to_next_different_frame(self, max_seek=10):
        """
        Seek forward until we find a frame different from the previous one.
        
        Args:
            max_seek: Maximum number of frames to seek forward before giving up
        
        Returns:
            (success, frame, frames_skipped): Whether a different frame was found, the frame, and how many frames were skipped
        """
        frames_skipped = 0
        for _ in range(max_seek):
            ret, frame = self.cap.read()
            if not ret:
                return False, None, frames_skipped
            
            frames_skipped += 1
            if not self._frames_identical(self.prev_frame, frame):
                return True, frame, frames_skipped
        
        # If we couldn't find a different frame, return the last one we read
        return True, frame, frames_skipped
    
    def read(self):
        """
        Read the next frame, automatically skipping duplicates if enabled.
        
        Returns:
            (success, frame): Whether frame was read successfully, and the frame
        """
        ret, frame = self.cap.read()
        if not ret:
            return False, None
        
        # Track frame number (increment after read)
        frames_read = 1
        
        if self.skip_duplicates and self.prev_frame is not None:
            if self._frames_identical(self.prev_frame, frame):
                # Frame is identical, seek to next different one
                ret, frame, frames_skipped = self._seek_to_next_different_frame()
                if not ret:
                    return False, None
                # Update frame count based on how many frames we skipped
                frames_read += frames_skipped
        
        self.current_frame_num += frames_read
        self.prev_frame = frame.copy() if frame is not None else None
        return True, frame
    
    def get_frame_number(self):
        """Get current frame number (0-indexed)."""
        return self.current_frame_num
    
    def set_frame_number(self, frame_num):
        """Seek to a specific frame number."""
        self.cap.set(cv2.CAP_PROP_POS_FRAMES, frame_num)
        self.current_frame_num = frame_num
        # Clear previous frame when seeking
        self.prev_frame = None
    
    def release(self):
        """Release the video capture."""
        self.cap.release()

