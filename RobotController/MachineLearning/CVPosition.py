from ultralytics import YOLO
import numpy as np
import mmap
from typing import List, Optional
import socket
import json
import cv2

SHARED_FILE_NAME = 'cv_pos_img'
SHAPE = (720, 720)
PORT = 11116
SCALE_RATIO = 0.75

print("Initializing socket")
# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
print("Socket initialized")

def send_results_to_rc(bounding_box: list, conf: float, frame_id: int, time_milliseconds: int):
    message = json.dumps({
        'bounding_box': bounding_box,
        'conf': conf,
        'frame_id': int(frame_id),
        'time_milliseconds': int(time_milliseconds)
    })

    try:
        # Send the message
        sock.sendto(message.encode(), ('localhost', PORT))
        # Uncomment the following line if you need to debug
        # print(f"Sent bounding box info to localhost:{PORT}")
    except Exception as e:
        print("Error sending to RC: " + str(e))

def get_mat(existing_shm) -> Optional[np.ndarray]:
    try:
        existing_shm.seek(0)
        # Calculate the expected size of the data
        expected_size = SHAPE[0] * SHAPE[1]
        # Read the data from shared memory
        data = existing_shm.read(expected_size)
        # Check the actual size of the data read
        if len(data) != expected_size:
            print(f"Data size mismatch: expected {expected_size}, got {len(data)}")
            return None
        # Create a NumPy array backed by the shared memory without copying
        np_array = np.frombuffer(data, dtype=np.uint8).reshape(SHAPE)
        print(f"Image shape after reading from shared memory: {np_array.shape}")
        return np_array
    except ValueError as e:
        print("Error accessing shared memory: ", e)
        return None

def run_inference_batch(model, imgs: List[np.ndarray]):
    # Preprocess images
    processed_imgs = []
    for img in imgs:
        # Resize the image
        new_width = int(SHAPE[1] * SCALE_RATIO)
        new_height = int(SHAPE[0] * SCALE_RATIO)
        img_resized = cv2.resize(img, (new_width, new_height))
        # If grayscale, convert to RGB by stacking
        if len(img_resized.shape) == 2:
            img_resized = np.stack((img_resized, img_resized, img_resized), axis=-1)
        processed_imgs.append(img_resized)
    # Pass the list of images directly
    try:
        results = model.predict(processed_imgs)
    except Exception as e:
        print("Error in model.predict: ", e)
        return None
    return results

def main():
    print("Initializing YOLO model")
    model = YOLO('train_yolo/model_2.0_faceoffs.pt')
    model.to('cuda')  # Move model to GPU
    # Uncomment the following line if your GPU supports half-precision
    model.half()  # Use half-precision

    print("YOLO model initialized")

    # Open the shared memory once
    try:
        existing_shm = mmap.mmap(-1, SHAPE[0] * SHAPE[1], tagname=SHARED_FILE_NAME, access=mmap.ACCESS_READ)
    except FileNotFoundError:
        print("Shared memory not found")
        return

    while True:
        # Get the image
        img = get_mat(existing_shm)

        if img is None:
            continue

        # Efficiently extract frame_id and time_milliseconds
        frame_id = int.from_bytes(img[0, :4].tobytes(), byteorder='little')
        time_milliseconds = int.from_bytes(img[0, 4:8].tobytes(), byteorder='little')

        # Prepare images for batch inference
        imgs = [img, cv2.flip(img, 1)]
        results = run_inference_batch(model, imgs)

        if results is None:
            continue

        result1 = results[0]
        result2 = results[1]

        # Initialize variables
        result = False
        max_conf = 0
        bounding_box = None

        # Process the first result
        if result1 and len(result1.boxes) > 0:
            confs = result1.boxes.conf
            max_conf1 = confs.max().item()
            idx1 = confs.argmax()
            bounding_box1 = result1.boxes.xywh[idx1].cpu().numpy() / SCALE_RATIO
            result = True
        else:
            max_conf1 = 0
            bounding_box1 = None

        # Process the second result (flipped image)
        if result2 and len(result2.boxes) > 0:
            confs = result2.boxes.conf
            max_conf2 = confs.max().item()
            idx2 = confs.argmax()
            bounding_box2 = result2.boxes.xywh[idx2].cpu().numpy() / SCALE_RATIO
            # Flip the bounding box back
            bounding_box2[0] = SHAPE[1] - bounding_box2[0]
            result = True
        else:
            max_conf2 = 0
            bounding_box2 = None

        # Combine results
        if bounding_box1 is not None and bounding_box2 is not None:
            max_conf = (max_conf1 + max_conf2) / 2
            bounding_box = (bounding_box1 + bounding_box2) / 2
        elif bounding_box1 is not None:
            max_conf = max_conf1
            bounding_box = bounding_box1
        elif bounding_box2 is not None:
            max_conf = max_conf2
            bounding_box = bounding_box2

        # Send results if detection was made
        if result and bounding_box is not None:
            bounding_box = bounding_box.tolist()
            send_results_to_rc(bounding_box, max_conf, frame_id, time_milliseconds)

        # Optionally, display the image (comment out to save time)
        """
        img_display = cv2.resize(img, (0, 0), fx=SCALE_RATIO, fy=SCALE_RATIO)
        if bounding_box is not None:
            x, y, w, h = bounding_box
            cv2.rectangle(img_display, (int(x - w/2), int(y - h/2)), (int(x + w/2), int(y + h/2)), (0, 255, 0), 2)
        cv2.imshow("image", img_display)
        cv2.waitKey(1)
        """

if __name__ == '__main__':
    main()
