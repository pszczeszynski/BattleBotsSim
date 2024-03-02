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
    # print(np_array)


    # # squash to gray
    # if np_array.shape[2] == 4:
    #     np_array = cv2.cvtColor(np_array, cv2.COLOR_RGBA2GRAY)

    # return the np array
    return np_array


print("Initializing socket")
# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
print("Socket initialized")

def send_results_to_rc(bounding_box: np.ndarray, conf: float):
    """
    Sends bounding box information to the robot controller via UDP.

    Parameters:
    - bounding_box: A numpy.ndarray containing bounding box information.
    - conf: the confidence
    """
    # Convert the bounding box numpy array to a list and then to JSON string
    bounding_box_list = bounding_box.tolist()[0]

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
    print("shape: ", img.shape)
    # if gray, stack to 3 channels
    if len(img.shape) == 2:
        img = np.stack((img, img, img), axis=2)
    
    results = model.predict(img, device=0)

    if not results or len(results) == 0:
        return False, None, None
    
    max_conf, bounding_box = 0, np.zeros(4)

    
    for r in results:
        if len(r.boxes.conf) == 0:
            return False, None, None
        if r.boxes.conf > max_conf:
            max_conf = r.boxes.conf
            bounding_box = r.boxes.xywh
    
    return True, max_conf, bounding_box

def main():
    print("Initializing YOLO model")
    model = YOLO('train_yolo/model4_orbitron_preprocessed_forks.pt')
    print("YOLO model initialized")

    while True:
        print("Getting image")
        # 1. get the imge
        img = get_mat(SHARED_FILE_NAME)
        # imshow
        cv2.imshow("Image", img)
        # wait key
        cv2.waitKey(1)
        print("Got image")
        # 2. retry if None
        if img is None:
            continue

        result, max_conf, bounding_box = run_inference(model, img)

        print("result: ", result, "max_conf: ", max_conf, "bounding_box: ", bounding_box)
        if result and max_conf > 0.5:
            send_results_to_rc(bounding_box, max_conf.item())

if __name__ == '__main__':
    main()