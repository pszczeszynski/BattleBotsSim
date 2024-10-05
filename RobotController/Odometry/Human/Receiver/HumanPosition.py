from enum import Enum
import socket
import cv2
import numpy as np

# Server configuration
HOST = '10.13.37.102'
PORT = 11118

POSITION = 0
OPPONENT_POSITION = 1
OPPONENT_ROTATION = 2
OPPONENT_ROTATION_VEL = 3

WINDOW_IMAGE_SIZE = 720
RAW_IMAGE_SIZE = 360

MAX_BUFFER_SIZE = 65507  # Maximum UDP payload size

MAX_RELATIVE_ADJUST_SPEED = 3.1415 * 2  # rad/s

class LastClickData:
    def __init__(self, position_clicked: tuple[int, int]=None, data_type: int=POSITION):
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

# Callback on mouse events
def mouse_callback(event, x, y, flags, param):
    global last_click

    total_width = 2 * WINDOW_IMAGE_SIZE
    if x < 0 or x > total_width or y < 0 or y > WINDOW_IMAGE_SIZE:
        return

    # Determine if the click is on the left or right image
    if x < WINDOW_IMAGE_SIZE:
        # Left image (cropped over opponent) - handle rotation on mouse hover
        if event == cv2.EVENT_MOUSEMOVE:
            last_click.data_type = OPPONENT_ROTATION
            last_click.position_clicked = (x, y)
    else:
        # Right image (full field) - handle position clicks
        x_adjusted = x - WINDOW_IMAGE_SIZE
        if event == cv2.EVENT_LBUTTONDOWN:
            # Left click sets the robot's position
            last_click.position_clicked = (x_adjusted, y)
            last_click.data_type = POSITION
        elif event == cv2.EVENT_RBUTTONDOWN:
            # Right click sets the opponent's position
            last_click.position_clicked = (x_adjusted, y)
            last_click.data_type = OPPONENT_POSITION

# Create a UDP socket
client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
print("UDP socket created.")

# Set up the window and callback
cv2.namedWindow("Streaming Image", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Streaming Image", 2 * WINDOW_IMAGE_SIZE, WINDOW_IMAGE_SIZE)
cv2.setMouseCallback("Streaming Image", mouse_callback)

def send_data_to_robot_controller():
    """
    Send the last click data to the robot controller.
    """
    data = last_click.data_type.to_bytes(4, byteorder='little') + \
           int(last_click.position_clicked[0] / 2).to_bytes(4, byteorder='little') + \
           int(last_click.position_clicked[1] / 2).to_bytes(4, byteorder='little')
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

        # Create the cropped image centered on the opponent
        def crop_image_around_robot(image, robot_pos, crop_size):
            x = robot_pos[0] - crop_size // 2
            y = robot_pos[1] - crop_size // 2

            x1 = x
            y1 = y
            x2 = x + crop_size
            y2 = y + crop_size

            height, width = image.shape[:2]

            # Calculate necessary padding
            pad_left = max(0, -x1)
            pad_right = max(0, x2 - width)
            pad_top = max(0, -y1)
            pad_bottom = max(0, y2 - height)

            # Pad the image with black pixels if needed
            image_padded = np.pad(
                image,
                ((pad_top, pad_bottom), (pad_left, pad_right), (0, 0)),
                mode='constant',
                constant_values=0
            )

            # Adjust coordinates due to padding
            x1_new = x1 + pad_left
            y1_new = y1 + pad_top

            # Crop the image
            cropped_image = image_padded[y1_new:y1_new + crop_size, x1_new:x1_new + crop_size]

            return cropped_image

        # Prepare the left image (cropped)
        left_image = crop_image_around_robot(received_image, (opponentPosX, opponentPosY), RAW_IMAGE_SIZE)
        left_image = cv2.resize(left_image, (WINDOW_IMAGE_SIZE, WINDOW_IMAGE_SIZE))

        # Prepare the right image (full field)
        right_image = cv2.resize(received_image.copy(), (WINDOW_IMAGE_SIZE, WINDOW_IMAGE_SIZE))

        # Combine images side by side
        combined_image = np.hstack((left_image, right_image))

        # Draw on the left image (cropped)
        # Relative positions for the cropped image
        relRobotPosX = robotPosX - opponentPosX
        relRobotPosY = robotPosY - opponentPosY

        # Scale factor for left image
        scale_factor_left = WINDOW_IMAGE_SIZE / RAW_IMAGE_SIZE

        screenRobotPosX_left = int((relRobotPosX + RAW_IMAGE_SIZE // 2) * scale_factor_left)
        screenRobotPosY_left = int((relRobotPosY + RAW_IMAGE_SIZE // 2) * scale_factor_left)

        screenOpponentPosX_left = WINDOW_IMAGE_SIZE // 2
        screenOpponentPosY_left = WINDOW_IMAGE_SIZE // 2

        # Draw the robot position on the left image
        cv2.circle(combined_image[:, :WINDOW_IMAGE_SIZE], (screenRobotPosX_left, screenRobotPosY_left), radius=20, color=(0, 255, 0), thickness=2)
        # Draw the opponent position on the left image
        cv2.circle(combined_image[:, :WINDOW_IMAGE_SIZE], (screenOpponentPosX_left, screenOpponentPosY_left), radius=20, color=(255, 0, 0), thickness=2)

        # Draw the opponent's rotation arrow on the left image
        opponentRotRad = np.radians(opponentAngleDeg)
        x_arrow = int(screenOpponentPosX_left + 50 * np.cos(opponentRotRad))
        y_arrow = int(screenOpponentPosY_left + 50 * np.sin(opponentRotRad))
        cv2.arrowedLine(combined_image[:, :WINDOW_IMAGE_SIZE], (screenOpponentPosX_left, screenOpponentPosY_left), (x_arrow, y_arrow), color=(0, 255, 0), thickness=2)

        # Draw on the right image (full field)
        # Scale factor for right image
        original_height, original_width = received_image.shape[:2]
        scale_factor_right_x = original_width / WINDOW_IMAGE_SIZE
        scale_factor_right_y = original_height / WINDOW_IMAGE_SIZE

        screenRobotPosX_right = int(robotPosX / scale_factor_right_x)
        screenRobotPosY_right = int(robotPosY / scale_factor_right_y)

        screenOpponentPosX_right = int(opponentPosX / scale_factor_right_x)
        screenOpponentPosY_right = int(opponentPosY / scale_factor_right_y)

        # Draw the robot position on the right image
        cv2.circle(combined_image[:, WINDOW_IMAGE_SIZE:], (screenRobotPosX_right, screenRobotPosY_right), radius=20, color=(0, 255, 0), thickness=2)
        # Draw the opponent position on the right image
        cv2.circle(combined_image[:, WINDOW_IMAGE_SIZE:], (screenOpponentPosX_right, screenOpponentPosY_right), radius=20, color=(255, 0, 0), thickness=2)

        # Draw the opponent's rotation arrow on the right image
        x_arrow_right = int(screenOpponentPosX_right + 50 * np.cos(opponentRotRad))
        y_arrow_right = int(screenOpponentPosY_right + 50 * np.sin(opponentRotRad))
        cv2.arrowedLine(combined_image[:, WINDOW_IMAGE_SIZE:], (screenOpponentPosX_right, screenOpponentPosY_right), (x_arrow_right, y_arrow_right), color=(0, 255, 0), thickness=2)

        # Handle last_click data for drawing
        if last_click.data_type == POSITION:
            # Draw 'X' on the right image
            x_draw = int(last_click.position_clicked[0])
            y_draw = int(last_click.position_clicked[1])
            draw_x(combined_image[:, WINDOW_IMAGE_SIZE:], (x_draw, y_draw), color=(0, 0, 255), size=20, thickness=2)
        elif last_click.data_type == OPPONENT_POSITION:
            # Draw circle on the right image
            x_draw = int(last_click.position_clicked[0])
            y_draw = int(last_click.position_clicked[1])
            cv2.circle(combined_image[:, WINDOW_IMAGE_SIZE:], (x_draw, y_draw), radius=20, color=(255, 0, 0), thickness=2)
        elif last_click.data_type == OPPONENT_ROTATION:
            # Draw arrow on the left image
            x_draw = int(last_click.position_clicked[0])
            y_draw = int(last_click.position_clicked[1])
            center_left = (WINDOW_IMAGE_SIZE // 2, WINDOW_IMAGE_SIZE // 2)
            cv2.arrowedLine(combined_image[:, :WINDOW_IMAGE_SIZE], center_left, (x_draw, y_draw), color=(0, 255, 0), thickness=2)
        else:
            print("Error: Invalid data type")

        send_data_to_robot_controller()

        # Display the combined image
        cv2.imshow("Streaming Image", combined_image)
        cv2.waitKey(1)

except KeyboardInterrupt:
    # Close the socket
    client_socket.close()
    # Close OpenCV window
    cv2.destroyAllWindows()
