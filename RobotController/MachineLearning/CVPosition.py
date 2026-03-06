"""
CVPosition Python inference bridge.

Protocol:
  - ready: Discovery only. Python announces presence. C++ uses once at startup.
  - frame_ready: C++ signals frame written. Python waits for this.
  - frame_copied: Python signals done copying. Only this allows C++ to write next frame.
  - result: Python sends inference result.

Python sends ready only while idle (waiting for frame_ready), never as frame ownership.
"""
from ultralytics import YOLO
import numpy as np
import mmap
import time
from typing import List, Optional, Tuple
import socket
import json
import cv2

SHARED_FILE_NAME = 'cv_pos_img'
SHARED_HEADER_SIZE = 12
SHAPE = (720, 720)
IMAGE_BYTES = SHAPE[0] * SHAPE[1]
SHARED_TOTAL_SIZE = SHARED_HEADER_SIZE + IMAGE_BYTES
PORT = 11116
SCALE_RATIO = 0.75

GET_MAT_MAX_RETRIES = 100
GET_MAT_SLEEP_S = 0.0001
READY_INTERVAL_S = 0.5
SHARED_MEM_RETRY_S = 1.0

RC_ADDR = ('localhost', PORT)

print("Initializing socket")
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(READY_INTERVAL_S)
print("Socket initialized")


def _send_ready() -> None:
    """Announce Python is ready. Supports C++-first or Python-first startup."""
    try:
        sock.sendto(
            json.dumps({"type": "ready"}).encode(),
            RC_ADDR,
        )
    except Exception as e:
        print("Error sending ready:", e)


def _send_frame_copied(frame_id: int, time_milliseconds: int) -> None:
    """Signal C++ that we have copied the frame (done reading)."""
    try:
        sock.sendto(
            json.dumps({
                "type": "frame_copied",
                "frame_id": int(frame_id),
                "time_milliseconds": int(time_milliseconds),
            }).encode(),
            RC_ADDR,
        )
    except Exception as e:
        print("Error sending frame_copied:", e)


def _send_result(
    bounding_box: list | str,
    conf: float,
    frame_id: int,
    time_milliseconds: int,
) -> None:
    """Send result with explicit type. Bounding box as float list when valid."""
    msg = {
        "type": "result",
        "frame_id": int(frame_id),
        "time_milliseconds": int(time_milliseconds),
        "bounding_box": bounding_box,
        "conf": float(conf),
    }
    try:
        sock.sendto(json.dumps(msg).encode(), RC_ADDR)
    except Exception as e:
        print("Error sending result:", e)


def _recv_frame_ready() -> Optional[Tuple[int, int]]:
    """Block until C++ signals frame_ready. Returns (frame_id, time_ms) or None on timeout."""
    try:
        data, _ = sock.recvfrom(512)
        msg = json.loads(data.decode())
        if msg.get("type") != "frame_ready":
            return None
        fid = msg.get("frame_id")
        tms = msg.get("time_milliseconds")
        if fid is None or tms is None:
            return None
        return (int(fid), int(tms))
    except (json.JSONDecodeError, TypeError, KeyError) as e:
        print("Invalid frame_ready message:", e)
        return None
    except socket.timeout:
        return None


def _wait_for_frame_ready() -> Optional[Tuple[int, int]]:
    """While idle: send ready, then recv frame_ready (timeout = READY_INTERVAL_S)."""
    while True:
        _send_ready()
        result = _recv_frame_ready()
        if result is not None:
            return result


def get_mat(existing_shm) -> Optional[Tuple[np.ndarray, int, int]]:
    """Seqlock read. Return (view, frame_id, time_ms) or None."""
    mv = memoryview(existing_shm)
    for _ in range(GET_MAT_MAX_RETRIES):
        try:
            seq1 = int.from_bytes(mv[0:4], 'little')
            if (seq1 & 1) != 0:
                time.sleep(GET_MAT_SLEEP_S)
                continue
            frame_id = int.from_bytes(mv[4:8], 'little')
            time_milliseconds = int.from_bytes(mv[8:12], 'little')
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


_processed_imgs: List[np.ndarray] = []


def run_inference_batch(model, imgs: List[np.ndarray]):
    _processed_imgs.clear()
    new_width = int(SHAPE[1] * SCALE_RATIO)
    new_height = int(SHAPE[0] * SCALE_RATIO)
    for img in imgs:
        img_resized = cv2.resize(img, (new_width, new_height))
        if len(img_resized.shape) == 2:
            img_resized = cv2.cvtColor(img_resized, cv2.COLOR_GRAY2BGR)
        _processed_imgs.append(img_resized)
    try:
        return model.predict(_processed_imgs)
    except Exception as e:
        print("Error in model.predict:", e)
        return None


def main():
    print("Initializing YOLO model")
    model = YOLO('train_yolo/model9_orbitron_v5.pt')
    model.to('cuda')
    model.half()
    print("YOLO model initialized")

    existing_shm = None
    while existing_shm is None:
        try:
            existing_shm = mmap.mmap(
                -1,
                SHARED_TOTAL_SIZE,
                tagname=SHARED_FILE_NAME,
                access=mmap.ACCESS_READ,
            )
        except FileNotFoundError:
            print("Shared memory not found, retrying...")
            time.sleep(SHARED_MEM_RETRY_S)
    print("Shared memory opened")

    last_frame_id: Optional[int] = None
    while True:
        ready_info = _wait_for_frame_ready()
        if ready_info is None:
            continue
        expected_frame_id, expected_time_ms = ready_info

        result = get_mat(existing_shm)
        if result is None:
            continue
        img, frame_id, time_milliseconds = result
        if frame_id != expected_frame_id or time_milliseconds != expected_time_ms:
            continue
        if frame_id == last_frame_id:
            time.sleep(0.001)
            continue
        last_frame_id = frame_id

        img_copy = np.copy(img)
        _send_frame_copied(frame_id, time_milliseconds)

        imgs = [img_copy, cv2.flip(img_copy, 1)]
        results = run_inference_batch(model, imgs)
        if results is None:
            continue

        result1, result2 = results[0], results[1]
        max_conf = 0.0
        bounding_box = None
        bounding_box1 = None
        max_conf1 = 0.0
        bounding_box2 = None
        max_conf2 = 0.0

        if result1 and len(result1.boxes) > 0:
            confs = result1.boxes.conf
            max_conf1 = confs.max().item()
            idx1 = confs.argmax()
            bounding_box1 = result1.boxes.xywh[idx1].cpu().numpy() / SCALE_RATIO

        if result2 and len(result2.boxes) > 0:
            confs = result2.boxes.conf
            max_conf2 = confs.max().item()
            idx2 = confs.argmax()
            bounding_box2 = result2.boxes.xywh[idx2].cpu().numpy() / SCALE_RATIO
            bounding_box2[0] = SHAPE[1] - bounding_box2[0]

        if bounding_box1 is not None and bounding_box2 is not None:
            max_conf = (max_conf1 + max_conf2) / 2
            bounding_box = (bounding_box1 + bounding_box2) / 2
        elif bounding_box1 is not None:
            max_conf = max_conf1
            bounding_box = bounding_box1
        elif bounding_box2 is not None:
            max_conf = max_conf2
            bounding_box = bounding_box2

        if bounding_box is not None:
            bbox_list = bounding_box.tolist()
            _send_result(bbox_list, max_conf, frame_id, time_milliseconds)
        else:
            _send_result("invalid", -1.0, frame_id, time_milliseconds)


if __name__ == '__main__':
    main()
