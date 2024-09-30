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
OPPONENT_ROTATION_VEL = 3

WINDOW_IMAGE_SIZE = 720
RAW_IMAGE_SIZE = 360

MAX_BUFFER_SIZE = 65507  # Maximum UDP payload size

MAX_RELATIVE_ADJUST_SPEED = 3.1415 * 2 # rad/s

class LastClickData:
    def __init__(self, position_clicked: tuple[int, int]=None, data_type:int=POSITION):
        self.position_clicked: tuple[int, int] = position_clicked
        self.data_type: int = data_type


# Function to draw an 'X' at a given position
def draw_x(img, center, color=(0, 0, 255), size=20, thickness=2):
    x, y = center
    x, y = int(x), int(y)
    cv2.line(img, (x - size, y - size), (x + size, y + size), color, thickness)
    cv2.line(img, (x - size, y + size), (x + size, y - size), color, thickness)

robotPosX = 0
robotPosY = 0
opponentPosX = 0
opponentPosY = 0
opponentAngleDeg = 0

last_click = LastClickData((0, 0), POSITION)
left_click_down = False


# Callback on mouse click
def mouse_callback(event, x, y, flags, param):
    global last_click
    global left_click_down
    # x = x - (X_WIDTH - SIDE_LENGTH) // 2
    # y = y - (Y_HEIGHT - SIDE_LENGTH) // 2
    # if x < 0 or x > SIDE_LENGTH or y < 0 or y > SIDE_LENGTH:
    if x < 0 or x > WINDOW_IMAGE_SIZE or y < 0 or y > WINDOW_IMAGE_SIZE:
        return


    pos_data = None
    if event == cv2.EVENT_LBUTTONDOWN:
        last_click.position_clicked = (x, y)
        last_click.data_type = OPPONENT_ROTATION_VEL
        left_click_down = True
    if event == cv2.EVENT_RBUTTONDOWN:
        last_click.position_clicked = (x, y)
        last_click.data_type = POSITION
        pos_data = POSITION.to_bytes(4, byteorder='little') + x.to_bytes(4, byteorder='little') + y.to_bytes(4, byteorder='little')
    if event == cv2.EVENT_MBUTTONDOWN:
        last_click.position_clicked = (x, y)
        last_click.data_type = OPPONENT_POSITION
        pos_data = OPPONENT_POSITION.to_bytes(4, byteorder='little') + x.to_bytes(4, byteorder='little') + y.to_bytes(4, byteorder='little')
    

    # if you released
    if event == cv2.EVENT_LBUTTONUP:
        left_click_down = False
        if last_click.data_type == OPPONENT_ROTATION_VEL:
            last_click.position_clicked = (0, 0) # reset the relative speed to 0

    # mouse moved => relatively adjust the angle at a set speed
    if event == cv2.EVENT_MOUSEMOVE:
        if left_click_down:
            last_click.position_clicked = (x, y)
            last_click.data_type = OPPONENT_ROTATION_VEL


    if pos_data is not None:
        for i in range(3):
            print(f"Sending data: {pos_data}")
            client_socket.sendto(pos_data, (HOST, PORT))

# Create a UDP socket
client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# client_socket.setblocking(False)

print("UDP socket created.")

# Set up the window and callback
cv2.namedWindow("Streaming Image", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Streaming Image", WINDOW_IMAGE_SIZE, WINDOW_IMAGE_SIZE)
cv2.setMouseCallback("Streaming Image", mouse_callback)


def send_data_to_robot_controller():
    """
    Send the last click data to the robot controller.
    """

    data = last_click.data_type.to_bytes(4, byteorder='little') + \
            last_click.position_clicked[0].to_bytes(4, byteorder='little') + \
                last_click.position_clicked[1].to_bytes(4, byteorder='little')
    client_socket.sendto(data, (HOST, PORT))


# Send a test message
client_socket.sendto(b'Hello', (HOST, PORT))
print("Test message sent.")
try:
    while True:
        # Receive data from the server
        try:
            data, server_address = client_socket.recvfrom(MAX_BUFFER_SIZE)
        except Exception as e:
            print("Error receiving data: " + str(e))
            send_data_to_robot_controller()
            continue

        if len(data) < 4:
            print("Error: Incomplete size data received")
            send_data_to_robot_controller()
            continue

        currMessagePos = 0
        robotPosX = int.from_bytes(data[currMessagePos:currMessagePos+4], byteorder='big')
        currMessagePos += 4
        robotPosY = int.from_bytes(data[currMessagePos:currMessagePos+4], byteorder='big')
        currMessagePos += 4
        opponentPosX = int.from_bytes(data[currMessagePos:currMessagePos+4], byteorder='big')
        currMessagePos += 4
        opponentPosY = int.from_bytes(data[currMessagePos:currMessagePos+4], byteorder='big')
        currMessagePos += 4
        opponentAngleDeg = int.from_bytes(data[currMessagePos:currMessagePos+4], byteorder='big')
        currMessagePos += 4
        imageSize = int.from_bytes(data[currMessagePos:currMessagePos+4], byteorder='big')
        currMessagePos += 4
        image_data = data[currMessagePos:]

        if len(image_data) < imageSize:
            print("Error: Incomplete image data received")
            continue

        # Convert the received data to a numpy array
        image_array = np.frombuffer(image_data[:imageSize], dtype=np.uint8)

        # Decode the numpy array to an image
        received_image = cv2.imdecode(image_array, cv2.IMREAD_COLOR)

        # Make a copy of the image to draw on
        display_image = received_image.copy()

        # draw the robot position
        cv2.circle(display_image, (int(robotPosX / 2), int(robotPosY / 2)), radius=20, color=(0, 255, 0), thickness=2)
        # draw the opponent position
        cv2.circle(display_image, (int(opponentPosX / 2), int(opponentPosY / 2)), radius=20, color=(255, 0, 0), thickness=2)
        opponentRotRad = np.radians(opponentAngleDeg)
        x = int(opponentPosX / 2 + 50 * np.cos(opponentRotRad))
        y = int(opponentPosY / 2 + 50 * np.sin(opponentRotRad))
        cv2.arrowedLine(display_image, (int(opponentPosX / 2), int(opponentPosY / 2)), (x, y), color=(0, 255, 0), thickness=2)


        if last_click.data_type == OPPONENT_ROTATION_VEL:
            # draw rect at the top at the same pos as the mouse relative speed
            x = last_click.position_clicked[0]
            y = 0
            cv2.rectangle(display_image, (x - 10, y), (x + 10, y + 20), color=(0, 0, 255), thickness=-1)


    
        # Draw 'X' at position_click
        if last_click.data_type == POSITION:
            draw_x(display_image, last_click.position_clicked, color=(0, 0, 255), size=20, thickness=2)

        # Draw circle at opponent_position_click
        elif last_click.data_type == OPPONENT_POSITION is not None:
            x, y = int(last_click.position_clicked[0]), int(last_click.position_clicked[1])
            cv2.circle(display_image, (x, y), radius=20, color=(255, 0, 0), thickness=2)

        # Draw arrow from center to opponent_rotation_click
        elif last_click.data_type == OPPONENT_ROTATION:
            center = (RAW_IMAGE_SIZE // 2, RAW_IMAGE_SIZE // 2)
            x, y = int(last_click.position_clicked[0]), int(last_click.position_clicked[1])
            cv2.arrowedLine(display_image, center, (x, y), color=(0, 255, 0), thickness=2)
    
        else:
            print("Error: Invalid data type")

        send_data_to_robot_controller()

        # # Send dummy data back to the server
        # dummy_data = 0
        # data_to_send = dummy_data.to_bytes(4, byteorder='little') + dummy_data.to_bytes(4, byteorder='big')
        # client_socket.sendto(data_to_send, server_address)

        # Display the received image with drawings
        cv2.imshow("Streaming Image", display_image)
        cv2.waitKey(1)


except KeyboardInterrupt:
    # Close the socket
    client_socket.close()
    # Close OpenCV window
    cv2.destroyAllWindows()
