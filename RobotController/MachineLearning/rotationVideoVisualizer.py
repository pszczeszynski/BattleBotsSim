import cv2
import numpy as np
from keras.models import Sequential
from keras.layers import Conv2D, MaxPooling2D, Flatten, Dense, Dropout
from keras.optimizers import Adam

from Visualization import network_output_to_angle
IMG_SIZE = (128, 128)

# Global variables for mouse position
mouse_x, mouse_y = 0, 0

def mouse_callback(event, x, y, flags, param):
    global mouse_x, mouse_y
    if event == cv2.EVENT_MOUSEMOVE:
        mouse_x, mouse_y = x, y


def load_model(weights_path):
    # Model Architecture
    model = Sequential([
        Conv2D(64, (3, 3), activation='relu', input_shape=(*IMG_SIZE, 1)),
        MaxPooling2D(2, 2),
        Conv2D(128, (3, 3), activation='relu'),
        MaxPooling2D(2, 2),
        Conv2D(256, (3, 3), activation='relu'),
        MaxPooling2D(2, 2),
        Flatten(),
        Dense(1024, activation='relu'),
        Dropout(0.5),
        Dense(1024, activation='relu'),
        Dropout(0.5),
        Dense(2) # 2 output neurons for cos and sin components
    ])

    # Now compile the model with this custom loss
    model.compile(optimizer=Adam(), loss="mse", metrics=['mae'])

    # load weights
    try:
        model.load_weights(weights_path)
        print("Loaded weights from file!")
    except OSError:
        print("No weights file found!")
    
    return model

def warp_frame_fixed_size(frame, src_points):
    """
    Warps the input frame given 4 corner points and returns a 720x720 image.

    Parameters:
    - frame: Input image (NumPy array).
    - src_points: List or array of 4 corner points in the order:
                  top-left, top-right, bottom-right, bottom-left.

    Returns:
    - warped_frame: The warped image of size 720x720 pixels.
    """
    # Ensure src_points is a NumPy array of type float32
    src = np.array(src_points, dtype='float32')

    # Define destination points to map the source points to a 720x720 image
    dst_size = 720
    dst = np.array([
        [0, 0],
        [dst_size - 1, 0],
        [dst_size - 1, dst_size - 1],
        [0, dst_size - 1]
    ], dtype='float32')

    # Compute the perspective transform matrix
    M = cv2.getPerspectiveTransform(src, dst)

    # Apply the warp perspective transform
    warped_frame = cv2.warpPerspective(frame, M, (dst_size, dst_size))

    return warped_frame

def main():
    # Load the video
    video_path = "C:/Users/Peter/Downloads/orbitron_tombclone.mkv"#'../Recordings/DisarrayFight2_clip.avi'  # Replace with your video path
    cap = cv2.VideoCapture(video_path)
    
    # Load the model
    model_path = 'rotationDetector_hoop.h5'  # Replace with your model path
    model = load_model(model_path)
    
    # Check if video opened successfully
    if not cap.isOpened():
        print("Error opening video file")
        return
    
    # Read the first frame to get dimensions
    ret, frame = cap.read()
    if not ret:
        print("Failed to read video")
        return

    # frame = warp_frame_fixed_size(frame,
    #  [(0, 0), (frame.shape[1], 0), (frame.shape[1], frame.shape[0]), (0, frame.shape[0])])

    # scale frame down 1.4
    # frame = cv2.resize(frame, (0, 0), fx=0.7, fy=0.7)

    frame_height, frame_width = frame.shape[:2]
    
    # Initialize mouse position to center
    global mouse_x, mouse_y
    mouse_x, mouse_y = frame_width // 2, frame_height // 2
    
    # Create a window and set the mouse callback
    cv2.namedWindow('Video')
    cv2.setMouseCallback('Video', mouse_callback)
    
    # Reset the video to the first frame
    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
    
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break  # End of video
        # frame = warp_frame_fixed_size(frame,
        #     [(frame.shape[1] * 0.25, 0), (frame.shape[1] * 0.75, 0), (frame.shape[1], frame.shape[0]), (0, frame.shape[0])])
        # scale frame down 1.4
        # frame = cv2.resize(frame, (0, 0), fx=0.7, fy=0.7)
        # Get current mouse position
        x_center, y_center = mouse_x, mouse_y
        
        # Crop a 128x128 section centered at the mouse position, pad with black if necessary
        crop_size = 128
        half_crop = crop_size // 2
        
        x_start = x_center - half_crop
        y_start = y_center - half_crop
        x_end = x_center + half_crop
        y_end = y_center + half_crop
        
        # Initialize the cropped image with zeros (black image)
        cropped_img = np.zeros((crop_size, crop_size, 3), dtype=np.uint8)
        
        # Compute the ranges for the source image and the cropped image
        x_start_src = max(0, x_start)
        y_start_src = max(0, y_start)
        x_end_src = min(frame_width, x_end)
        y_end_src = min(frame_height, y_end)
        
        x_start_dst = max(0, -x_start)
        y_start_dst = max(0, -y_start)
        x_end_dst = crop_size - max(0, x_end - frame_width)
        y_end_dst = crop_size - max(0, y_end - frame_height)
        
        # Copy the region from the source frame to the cropped image
        cropped_img[y_start_dst:y_end_dst, x_start_dst:x_end_dst] = frame[y_start_src:y_end_src, x_start_src:x_end_src]
        
        # Convert to grayscale
        gray_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2GRAY)
        
        # Normalize the image
        gray_img = gray_img.astype('float32') / 255.0
        
        # Expand dimensions to match model input shape (batch_size, height, width, channels)
        input_img = np.expand_dims(gray_img, axis=0)
        input_img = np.expand_dims(input_img, axis=-1)  # Add channel dimension
        
        # Predict using the model
        prediction = model.predict(input_img)

        angle = network_output_to_angle(prediction)

        cos_theta = np.cos(angle)
        sin_theta = np.sin(angle)
        
        # Compute arrow endpoint
        arrow_length = 50  # Adjust as needed
        arrow_x = int(x_center + arrow_length * cos_theta)
        arrow_y = int(y_center + arrow_length * sin_theta)
        
        # Draw the arrow on the frame
        cv2.arrowedLine(frame, (x_center, y_center), (arrow_x, arrow_y), (0, 0, 255), 2)
        
        # Display the frame
        cv2.imshow('Video', frame)
        
        # Exit on 'q' or 'Esc' key
        key = cv2.waitKey(1)
        if key == ord('q') or key == 27:
            break
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
