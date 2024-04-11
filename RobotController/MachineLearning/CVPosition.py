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
SCALE_RATIO = 1.0

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
        existing_shm = mmap.mmap(-1, SHAPE[0] * SHAPE[1], tagname=SHARED_FILE_NAME, access=mmap.ACCESS_READ)

        # Read data from shared memory
        data = existing_shm.read(SHAPE[0] * SHAPE[1])
    except FileNotFoundError:
        return None

    # Create a NumPy array backed by the shared memory
    # Note: The total size is calculated by multiplying the dimensions of the shape
    np_array = np.ndarray(SHAPE, dtype=np.uint8, buffer=data)

    # return the np array
    return np_array


print("Initializing socket")
# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
print("Socket initialized")

def send_results_to_rc(bounding_box: np.ndarray, conf: float, frame_id: int, time_milliseconds: int):
    """
    Sends bounding box information to the robot controller via UDP.

    Parameters:
    - bounding_box: A numpy.ndarray containing bounding box information.
    - conf: the confidence
    - frame_id: the frame id (extracted from the first pixel)
    - time_milliseconds: the time in milliseconds (extracted from the next 4 pixels)
    """
    # Convert the bounding box numpy array to a list and then to JSON string
    bounding_box_list: List = bounding_box.tolist()

    print("Bounding box: ", bounding_box_list)
    print("Confidence: ", conf)

    message = json.dumps({'bounding_box': bounding_box_list, 'conf': conf, 'frame_id': int(frame_id), 'time_milliseconds': int(time_milliseconds)})

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
    
    print("image shape: " + str(img.shape))
    
    try:
        results = model.predict(img)[0]
    except Exception as e:
        print("Error in model.predict: ", e)
        return False, None, None

    if not results or len(results) == 0:
        return False, None, None
    
    max_conf, bounding_box = 0, np.zeros(4)

    try:
        for i in range(len(results.boxes.conf)):
            if results.boxes.conf[i] > max_conf:
                max_conf = results.boxes.conf[i].item()
                bounding_box = results.boxes.xywh[i] / SCALE_RATIO
    except Exception as e:
        print("Error in run_inference: ", e)
        return False, None, None
    return True, max_conf, bounding_box

def main():
    print("Initializing YOLO model")
    model = YOLO('train_yolo/model4_orbitron_preprocessed_forks.pt')
    print("YOLO model initialized")

    while True:
        print("Getting image")
        # 1. get the imge
        img = get_mat(SHARED_FILE_NAME)

        # the first 4 pixels are the frame id
        frame_id = 0
        for i in range(4):
            frame_id += img[0, i] << (8 * i)

        # parse the time from the next 4 pixels
        time_milliseconds = 0
        for i in range(4):
            time_milliseconds += img[0, i + 4] << (8 * i)

        print("id: ", frame_id)
        print("Got image")
        # 2. retry if None
        if img is None:
            continue

        result, max_conf, bounding_box = run_inference(model, img)

        # draw the bounding box

        print("result: ", result, "max_conf: ", max_conf, "bounding_box: ", bounding_box)
        if result:
            bb = bounding_box.tolist()
            print("bb: ", bb)
            if (len(bb) < 4):
                print("continuing")
                continue

            # draw circle at bb[0] and bb[1]

            # img = cv2.rectangle(img, (int(bb[0] - bb[2] / 2), int(bb[1] - bb[3] / 2)), (int(bb[0] + bb[2] / 2), int(bb[1] + bb[3] / 2)), (0, 255, 0), 2)
            # cv2.imshow("Bounding Box", img)
            # cv2.waitKey(1)

            # print("bb[0]: " + str(bb[0]))

            print("bounding box: ", bounding_box)
            send_results_to_rc(bounding_box, max_conf, frame_id, time_milliseconds)

if __name__ == '__main__':
    main()