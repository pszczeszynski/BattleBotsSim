from enum import Enum
import socket
import cv2
import numpy as np

# Server configuration
HOST = 'localhost'
PORT = 11118

POSITION = 0
OPPONENT_POSITION = 1
OPPONENT_ROTATION = 2

X_WIDTH = 1280
Y_HEIGHT = 720
SIDE_LENGTH = 720

MAX_BUFFER_SIZE = 65507  # Maximum UDP payload size

# Callback on mouse click
def mouse_callback(event, x, y, flags, param):
    x = x - (X_WIDTH - SIDE_LENGTH) // 2
    y = y - (Y_HEIGHT - SIDE_LENGTH) // 2
    if x < 0 or x > SIDE_LENGTH or y < 0 or y > SIDE_LENGTH:
        return
    if event == cv2.EVENT_LBUTTONDOWN:
        print(f"Mouse clicked at (x={x}, y={y}), Mode = {POSITION}")
        pos_data = POSITION.to_bytes(4, byteorder='big') + x.to_bytes(4, byteorder='big') + y.to_bytes(4, byteorder='big')
        client_socket.sendto(pos_data, (HOST, PORT))
    if event == cv2.EVENT_RBUTTONDOWN:
        print(f"Mouse clicked at (x={x}, y={y}), Mode = {OPPONENT_POSITION}")
        pos_data = OPPONENT_POSITION.to_bytes(4, byteorder='big') + x.to_bytes(4, byteorder='big') + y.to_bytes(4, byteorder='big')
        client_socket.sendto(pos_data, (HOST, PORT))
    if event == cv2.EVENT_MBUTTONDOWN:
        print(f"Mouse clicked at (x={x}, y={y}), Mode = {OPPONENT_ROTATION}")
        pos_data = OPPONENT_ROTATION.to_bytes(4, byteorder='big') + x.to_bytes(4, byteorder='big') + y.to_bytes(4, byteorder='big')
        client_socket.sendto(pos_data, (HOST, PORT))

# Create a UDP socket
client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
print("UDP socket created.")

# Set up the window and callback
cv2.namedWindow("Streaming Image", cv2.WINDOW_NORMAL)
cv2.setMouseCallback("Streaming Image", mouse_callback)

# Send a test message
client_socket.sendto(b'Hello', (HOST, PORT))
try:
    while True:
        print("Waiting for image data...")
        # Receive data from the server
        data, server_address = client_socket.recvfrom(MAX_BUFFER_SIZE)
        if not data:
            print("Error: No data received")
            break
        print("Received data length:", len(data))

        # The first 4 bytes are the size of the image data
        if len(data) < 4:
            print("Error: Incomplete size data received")
            continue
        size_data = data[:4]
        size = int.from_bytes(size_data, byteorder='big')
        print("Size: ", size)
        image_data = data[4:]

        if len(image_data) < size:
            print("Error: Incomplete image data received")
            continue

        # Convert the received data to a numpy array
        image_array = np.frombuffer(image_data[:size], dtype=np.uint8)
        # Decode the numpy array to an image
        received_image = cv2.imdecode(image_array, cv2.IMREAD_COLOR)

        # Send dummy data back to the server
        dummy_data = 0
        data_to_send = dummy_data.to_bytes(4, byteorder='big') + dummy_data.to_bytes(4, byteorder='big')
        client_socket.sendto(data_to_send, server_address)

        # Display the received image
        cv2.imshow("Streaming Image", received_image)
        cv2.waitKey(1)

except KeyboardInterrupt: 
    # Close the socket
    client_socket.close()
    # Close OpenCV window
    cv2.destroyAllWindows()
