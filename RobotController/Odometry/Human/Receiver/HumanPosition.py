from enum import Enum
import socket
import cv2
import numpy as np

# Server configuration
HOST = '127.0.0.1'
PORT = 11117

class mode(Enum):
    POSITION = 0
    ROTATION = 1

POSITION = 0
OPPONENT_POSITION = 1
OPPONENT_ROTATION = 2

X_WIDTH = 1280
Y_HEIGHT = 720
SIDE_LENGTH = 720

# Callback on mouse click
def mouse_callback(event, x, y, flags, param):
    x = x - (X_WIDTH - SIDE_LENGTH) // 2
    y = y - (Y_HEIGHT - SIDE_LENGTH) // 2
    if x < 0 or x > SIDE_LENGTH or y < 0 or y > SIDE_LENGTH:
        return
    if event == cv2.EVENT_LBUTTONDOWN:
        print(f"Mouse clicked at (x={x}, y={y}), Mode = {POSITION}")
        pos_data = POSITION.to_bytes(4, byteorder='big') + x.to_bytes(4, byteorder='big') + y.to_bytes(4, byteorder='big')
        client_socket.sendall(pos_data)
    if event == cv2.EVENT_RBUTTONDOWN:
        print(f"Mouse clicked at (x={x}, y={y}), Mode = {OPPONENT_POSITION}")
        pos_data = OPPONENT_POSITION.to_bytes(4, byteorder='big') + x.to_bytes(4, byteorder='big') + y.to_bytes(4, byteorder='big')
        client_socket.sendall(pos_data)
    if event == cv2.EVENT_MBUTTONDOWN:
        print(f"Mouse clicked at (x={x}, y={y}), Mode = {OPPONENT_ROTATION}")
        pos_data = OPPONENT_ROTATION.to_bytes(4, byteorder='big') + x.to_bytes(4, byteorder='big') + y.to_bytes(4, byteorder='big')
        client_socket.sendall(pos_data)

# Connect to the server
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((HOST, PORT))
print("Connected to the server.")

# Set up the window and callback
cv2.namedWindow("Streaming Image", cv2.WINDOW_NORMAL)
cv2.setMouseCallback("Streaming Image", mouse_callback)

try:
    while True:
        # Receive image data size from the server
        size_data = client_socket.recv(4)
        if not size_data:
            break
        size = int.from_bytes(size_data, byteorder='big')

        # Receive image data from the server
        image_data = b''
        while len(image_data) < size:
            remaining_bytes = size - len(image_data)
            data = client_socket.recv(remaining_bytes)
            if not data:
                break
            image_data += data
        if len(image_data) < size:
            print("Error: Incomplete data received")
            break

        # Convert the received data to a numpy array
        image_array = np.frombuffer(image_data, dtype=np.uint8)

        # Decode the numpy array to an image
        received_image = cv2.imdecode(image_array, cv2.IMREAD_COLOR)

        dummy_data = 0
        data_to_send = dummy_data.to_bytes(4, byteorder='big') + dummy_data.to_bytes(4, byteorder='big')
        client_socket.sendall(data_to_send)

        # Display the received image
        cv2.imshow("Streaming Image", received_image)
        cv2.waitKey(1)

except KeyboardInterrupt: 
    # Close the connection
    client_socket.close()

    # Close OpenCV window
    cv2.destroyAllWindows()
