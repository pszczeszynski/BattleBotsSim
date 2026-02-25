from ultralytics import YOLO
import numpy as np
import mmap
import time
from typing import List, Optional, Tuple
import socket
import json
import cv2

SHARED_FILE_NAME = 'cv_pos_img'
# Layout matches C++ SharedImageHeader: [seq:4][frame_id:4][time_ms:4] then image.
SHARED_HEADER_SIZE = 12
SHAPE = (720, 720)
IMAGE_BYTES = SHAPE[0] * SHAPE[1]
SHARED_TOTAL_SIZE = SHARED_HEADER_SIZE + IMAGE_BYTES
PORT = 11116
SCALE_RATIO = 0.75

# Seqlock retry: avoid spinning at 100% CPU when writer is mid-update.
GET_MAT_MAX_RETRIES = 100
GET_MAT_SLEEP_S = 0.0001  # 0.1 ms

print("Initializing socket")
# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
print("Socket initialized")

def send_results_to_rc(bounding_box: list | str, conf: float, frame_id: int, time_milliseconds: int):
    message = json.dumps({
        'bounding_box': bounding_box,
        'conf': conf,
        'frame_id': int(frame_id),
        'time_milliseconds': int(time_milliseconds)
    })

    print("sending: " + message)

    try:
        # Send the message
        sock.sendto(message.encode(), ('localhost', PORT))
        # Uncomment the following line if you need to debug
        # print(f"Sent bounding box info to localhost:{PORT}")
    except Exception as e:
        print("Error sending to RC: " + str(e))

def get_mat(existing_shm) -> Optional[Tuple[np.ndarray, int, int]]:
    """Seqlock read: use memoryview slices only (no .tobytes() — that would copy). Return (view, frame_id, time_ms) or None.
    Accept only when seq1 == seq2 and seq2 is even (writer finished). Early-out on odd seq1 to avoid parsing mid-write.
    """
    mv = memoryview(existing_shm)
    for _ in range(GET_MAT_MAX_RETRIES):
        try:
            # Read seq first; pass memoryview slice directly to avoid .tobytes() copy.
            seq1 = int.from_bytes(mv[0:4], 'little')
            # Early-out: if seq is odd, writer is mid-update — skip header/image parse and retry (saves CPU + avoids torn read).
            if (seq1 & 1) != 0:
                time.sleep(GET_MAT_SLEEP_S)
                continue
            frame_id = int.from_bytes(mv[4:8], 'little')
            time_milliseconds = int.from_bytes(mv[8:12], 'little')
            # Image as zero-copy view into the mmap.
            img = np.ndarray(
                shape=SHAPE,
                dtype=np.uint8,
                buffer=existing_shm,
                offset=SHARED_HEADER_SIZE,
            )
            seq2 = int.from_bytes(mv[0:4], 'little')
            if seq1 == seq2 and (seq2 & 1) == 0:
                return (img, frame_id, time_milliseconds)
        except (ValueError, TypeError) as e:
            print("Error accessing shared memory:", e)
            return None
        time.sleep(GET_MAT_SLEEP_S)
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
    model = YOLO('train_yolo/model9_orbitron_v5.pt') #    #train_yolo/model8_orbitron_preprocessed_hoop2.pt
    model.to('cuda')  # Move model to GPU
    model.half()  # Use half-precision

    print("YOLO model initialized")

    # Open the shared memory once (header + image)
    try:
        existing_shm = mmap.mmap(-1, SHARED_TOTAL_SIZE, tagname=SHARED_FILE_NAME, access=mmap.ACCESS_READ)
    except FileNotFoundError:
        print("Shared memory not found")
        return

    while True:
        # Get a consistent snapshot (img, frame_id, time_milliseconds) or None
        result = get_mat(existing_shm)
        if result is None:
            continue
        img, frame_id, time_milliseconds = result

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
            bounding_box_list = bounding_box.tolist()
            send_results_to_rc(bounding_box_list, max_conf, frame_id, time_milliseconds)
        else:
            send_results_to_rc("invalid", -1, frame_id, time_milliseconds)

        # Draw prediction and display
        if len(img.shape) == 2:
            display = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        else:
            display = img.copy()
        if bounding_box is not None:
            x, y, w, h = bounding_box
            x, y, w, h = int(x), int(y), int(w), int(h)
            cv2.rectangle(display, (x - w // 2, y - h // 2), (x + w // 2, y + h // 2), (0, 255, 0), 2)
            cv2.circle(display, (x, y), 4, (0, 255, 0), -1)
        # cv2.imshow("CVPosition", display)
        # cv2.waitKey(1)

if __name__ == '__main__':
    main()
