from ultralytics import YOLO
import numpy as np
import mmap
from typing import Optional
import socket
import json
import cv2

SHARED_FILE_NAME = 'cv_pos_img'
SHAPE = (720, 720)
PORT = 11116
SCALE_RATIO = 0.5
CONFIDENCE_THRESHOLD = 0

def get_mat(name) -> Optional[np.ndarray]:
    """
    Retrieve an OpenCV Mat from shared memory.

    Parameters:
    - name: The unique name of the shared memory block.

    Returns:
    - An np array containing the image data or None if couldn't find the memory
    """

    data= None
    try:
        # Access the existing shared memory block
        # Windows-specific way to open shared memory
        existing_shm = mmap.mmap(-1, SHAPE[0] * SHAPE[1], tagname=name, access=mmap.ACCESS_READ)

        # Read data from shared memory
        data = existing_shm.read(SHAPE[0] * SHAPE[1])
    except FileNotFoundError:
        return None

    # Create a NumPy array backed by the shared memory
    # Note: The total size is calculated by multiplying the dimensions of the shape
    np_array = np.ndarray(SHAPE, dtype=np.uint8, buffer=data)

    # return the np array
    return np_array

def send_results_to_rc(sock, bounding_box: np.ndarray, conf: float):
    """
    Sends bounding box information to the robot controller via UDP.

    Parameters:
    - bounding_box: A numpy.ndarray containing bounding box information.
    - conf: the confidence
    """
    # Convert the bounding box numpy array to a list and then to JSON string
    bounding_box_list = bounding_box.tolist()

    print("Bounding box: ", bounding_box_list)
    print("Confidence: ", conf)

    message = json.dumps({'bounding_box': bounding_box_list, 'conf': conf})

    try:
        # Send the message
        sock.sendto(message.encode(), ('localhost', PORT))
        print(f"Sent bounding box info to localhost:{PORT}")
    except Exception as e:
        print("Error sending to RC: " + str(e))


def run_inference(model, img: np.ndarray):
    shrunk_img = cv2.resize(img, (int(SHAPE[0]*SCALE_RATIO), int(SHAPE[1]*SCALE_RATIO)))
    img = np.zeros(img.shape, dtype=np.uint8)
    img[:int(SHAPE[0]*SCALE_RATIO), :int(SHAPE[1]*SCALE_RATIO)] = shrunk_img

    # if gray, stack to 3 channels
    if len(img.shape) == 2:
        img = np.stack((img, img, img), axis=2)
    
    results = model.predict(img, device=0)[0]

    if not results or len(results) == 0 or len(results.boxes.conf) == 0:
        return False, None, None
    
    max_conf, bounding_box = 0, np.zeros(4)

    try:
        for i in range(len(results.boxes.conf)):
            if results.boxes.conf[i] > max_conf:
                max_conf = results.boxes.conf[i].item()
                bounding_box = results.boxes.xywh[i] / SCALE_RATIO
    except:
        return False, None, None
    
    return True, max_conf, bounding_box

def main():
    print("Initializing socket")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print("Socket initialized")

    print("Initializing YOLO model")
    model = YOLO('train_yolo/model4_orbitron_preprocessed_forks.pt')
    print("YOLO model initialized")

    while True:
        # 1. get the imge
        print("Getting image")
        img = get_mat(SHARED_FILE_NAME)
        print("Got image")

        # imshow
        # cv2.imshow("Image", img)
        # wait key
        # cv2.waitKey(1)

        # 2. retry if None
        if img is None:
            continue

        result, max_conf, bounding_box = run_inference(model, img)
        print("result: ", result, "max_conf: ", max_conf, "bounding_box: ", bounding_box)
        if result and max_conf > CONFIDENCE_THRESHOLD:
            send_results_to_rc(sock, bounding_box, max_conf)

if __name__ == '__main__':
    main()