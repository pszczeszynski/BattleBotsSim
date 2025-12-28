import cv2

# ----------------------------
# Config (you can hardcode ROI here if you prefer)
# ROI format: (x, y, w, h) in pixels. Set to None to pick with mouse.
# ----------------------------
VIDEO_PATH = "C:/Dev/BattleBotsSim/RobotController/Recordings/disarray_overhead_trimmed.mp4"
START_ROI = (1500, 475, 140, 140)

# Default enabled trackers (toggle live with keys)
ENABLE_LK_FLOW = True       # key: 1
ENABLE_TEMPLATE = False     # key: 2
ENABLE_ORB_H = False         # key: 3

# Visualization
DRAW_FEATURES = True
SHOW_DEBUG_TEXT = True

# LK settings
LK_MAX_CORNERS = 200
LK_QUALITY_LEVEL = 0.01
LK_MIN_DISTANCE = 6
LK_WIN_SIZE = (21, 21)
LK_MAX_LEVEL = 3
LK_NUM_PAIRS = 30  # Number of random point pairs to use for rotation estimation

# Frame diff mask settings
FRAME_DIFF_THRESHOLD = 30  # Pixel intensity difference threshold
FRAME_DIFF_BLUR_SIZE = 5  # Gaussian blur kernel size (must be odd)
FRAME_DIFF_RE_THRESHOLD = 20  # Re-threshold value after blurring (lower = more lenient)
MOVEMENT_CHECK_RADIUS = 50  # Radius around center to check for movement
MOVEMENT_CHECK_THRESHOLD = 0.1  # Minimum fraction of mask pixels in radius to trigger filtering

# ORB settings
ORB_NFEATURES = 1000

# Template match settings
TM_METHOD = cv2.TM_CCOEFF_NORMED

