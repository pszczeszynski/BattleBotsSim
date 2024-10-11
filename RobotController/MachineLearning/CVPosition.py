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

def send_results_to_rc(bounding_box: list, conf: float, frame_id: int, time_milliseconds: int):
    """
    Sends bounding box information to the robot controller via UDP.

    Parameters:
    - bounding_box: A numpy.ndarray containing bounding box information.
    - conf: the confidence
    - frame_id: the frame id (extracted from the first pixel)
    - time_milliseconds: the time in milliseconds (extracted from the next 4 pixels)
    """
    # Convert the bounding box numpy array to a list and then to JSON string
    bounding_box_list: List = bounding_box

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
        results = model.predict(img, device='cuda')[0]
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
    model = YOLO('train_yolo/model_2.0_faceoffs.pt')
    model.to('cuda')  # Move model to GPU

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

        result1, max_conf1, bounding_box1 = run_inference(model, img)

        flipped_img = cv2.flip(img, 1)
        # re-run inference on a horizontal flip
        result2, max_conf2, bounding_box2 = run_inference(model, flipped_img)

        result = None
        max_conf = None
        bounding_box = None
        def flip_boundingbox(bounding_box):
            # flip
            bounding_box[0] = SHAPE[1] - bounding_box[0]
            return bounding_box

        if result1:
            bounding_box1 = bounding_box1.tolist()

        if result2:
            bounding_box2 = flip_boundingbox(bounding_box2.tolist())

        result = None
        max_conf = None
        bounding_box = None
        if result1 and not result2:
            result = result1
            max_conf = max_conf1
            bounding_box = bounding_box1
        elif result2 and not result1:
            result = result2
            max_conf = max_conf2
            bounding_box = bounding_box2
        elif result1 and result2:
            # average
            result = True
            max_conf = (max_conf1 + max_conf2) / 2
            # average the 2 lists
            bounding_box = [(bounding_box1[i] + bounding_box2[i]) / 2 for i in range(4)]
            
        
        # show both predictions
        img = cv2.resize(img, (0, 0), fx=SCALE_RATIO, fy=SCALE_RATIO)
        # draw
        if result1:
            cv2.rectangle(img, (int(bounding_box1[0]), int(bounding_box1[1])), (int(bounding_box1[0] + bounding_box1[2]), int(bounding_box1[1] + bounding_box1[3])), (0, 255, 0), 2)
        # if result2:
        #     cv2.rectangle(img, (int(bounding_box2[0]), int(bounding_box2[1])), (int(bounding_box2[0] + bounding_box2[2]), int(bounding_box2[1] + bounding_box2[3])), (0, 255, 0), 2)
        
        # show
        cv2.imshow("image", img)
        cv2.waitKey(1)
        
        # draw the bounding box

        print("result: ", result, "max_conf: ", max_conf, "bounding_box: ", bounding_box)
        if result1:
            bb = bounding_box
            print("bb: ", bb)
            if (len(bb) < 4):
                print("continuing")
                continue

            send_results_to_rc(bounding_box, max_conf, frame_id, time_milliseconds)
            print("bounding box: ", bounding_box)

if __name__ == '__main__':
    main()