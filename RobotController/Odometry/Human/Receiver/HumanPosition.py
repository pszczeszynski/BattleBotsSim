from enum import Enum
import socket
import cv2
import numpy as np
import threading
from PIL import Image, ImageTk
import tkinter as tk
from tkinter import ttk
from tkinter import HORIZONTAL

# Removed hardcoded HOST and PORT
# HOST = 'localhost'
# PORT = 11118

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


# Initialize client_socket as None
client_socket = None

# Function to send data to robot controller
def send_data_to_robot_controller():
    global client_socket
    """
    Send the last click data and UI state to the robot controller.

    Data order:
    1. Data type (4 bytes)
    2. X position (4 bytes)
    3. Y position (4 bytes)
    4. Foreground min delta (4 bytes)
    5. Background heal rate (4 bytes)
    6. Force Pos on click (4 bytes)
    7. Force Heal (4 bytes)
    """
    # Collect UI state
    foreground_min_delta_value = int(foreground_min_delta_slider.get())
    background_heal_rate_value = int(background_heal_rate_slider.get())
    force_pos_value = int(force_pos_var.get())
    force_heal_value = int(force_heal_var.get())

    # Prepare data to send
    # For example, pack the data into bytes
    data = last_click.data_type.to_bytes(4, byteorder='little') + \
           int(last_click.position_clicked[0] / 2).to_bytes(4, byteorder='little') + \
           int(last_click.position_clicked[1] / 2).to_bytes(4, byteorder='little') + \
           foreground_min_delta_value.to_bytes(4, byteorder='little') + \
           background_heal_rate_value.to_bytes(4, byteorder='little') + \
           force_pos_value.to_bytes(4, byteorder='little') + \
           force_heal_value.to_bytes(4, byteorder='little')

    print("sending click data: ", last_click.data_type, last_click.position_clicked)
    if client_socket is not None:
        client_socket.sendto(data, (HOST, PORT))
    else:
        print("Not connected yet.")


robotPosX = 0
robotPosY = 0
opponentPosX = 0
opponentPosY = 0
opponentAngleDeg = 0

last_click = LastClickData((0, 0), POSITION)

# Create the main GUI window
root = tk.Tk()
root.title("Streaming Image with Custom UI")

# Main panel
main_panel = tk.Frame(root)
main_panel.grid(row=0, column=0)

# Create a placeholder for the OpenCV image
image_label = tk.Label(main_panel)
image_label.grid(row=0, column=0)

# Side panel
side_panel = tk.Frame(root)
side_panel.grid(row=0, column=1, padx=10, pady=10)

# IP address and port input fields
ip_label = tk.Label(side_panel, text="IP Address:")
ip_label.grid(row=0, column=0, padx=5, pady=5)
ip_entry = tk.Entry(side_panel)
ip_entry.insert(0, "localhost")  # Default value
ip_entry.grid(row=0, column=1, padx=5, pady=5)

port_label = tk.Label(side_panel, text="Port:")
port_label.grid(row=1, column=0, padx=5, pady=5)
port_entry = tk.Entry(side_panel)
port_entry.insert(0, "11118")  # Default value
port_entry.grid(row=1, column=1, padx=5, pady=5)

# Connect button
def connect_to_server():
    global HOST, PORT, client_socket

    HOST = ip_entry.get()
    PORT = int(port_entry.get())

    # Create a UDP socket
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print("UDP socket created.")

    # Send a test message
    client_socket.sendto(b'Hello', (HOST, PORT))
    print("Test message sent.")

    # Start the receive thread
    threading.Thread(target=receive_thread, daemon=True).start()

connect_button = tk.Button(side_panel, text="Connect", command=connect_to_server)
connect_button.grid(row=2, column=0, columnspan=2, padx=5, pady=5)

# Auto L and Auto R buttons
auto_l_button = tk.Button(side_panel, text="Auto L", bg="lightblue", width=10)
auto_r_button = tk.Button(side_panel, text="Auto R", bg="red", width=10)
auto_l_button.grid(row=3, column=0, padx=5, pady=5)
auto_r_button.grid(row=3, column=1, padx=5, pady=5)

# Sliders with values displayed
foreground_label = tk.Label(side_panel, text="Foreground min delta:")
foreground_label.grid(row=4, column=0, columnspan=2, padx=5, pady=5)

foreground_min_delta_label = tk.Label(side_panel, text="0")
foreground_min_delta_label.grid(row=5, column=0, sticky="w", padx=5)
foreground_min_delta_slider = ttk.Scale(side_panel, from_=0, to=100, orient=HORIZONTAL)
foreground_min_delta_slider.grid(row=5, column=0, columnspan=2, padx=5, pady=5)

max_foreground_min_delta_label = tk.Label(side_panel, text="100")
max_foreground_min_delta_label.grid(row=5, column=1, sticky="e", padx=5)

background_label = tk.Label(side_panel, text="Background Heal Rate:")
background_label.grid(row=6, column=0, columnspan=2, padx=5, pady=5)

background_min_label = tk.Label(side_panel, text="0")
background_min_label.grid(row=7, column=0, sticky="w", padx=5)
background_heal_rate_slider = ttk.Scale(side_panel, from_=0, to=100, orient=HORIZONTAL)
background_heal_rate_slider.grid(row=7, column=0, columnspan=2, padx=5, pady=5)

max_background_heal_rate_label = tk.Label(side_panel, text="100")
max_background_heal_rate_label.grid(row=7, column=1, sticky="e", padx=5)

# Checkboxes
force_pos_var = tk.IntVar()
force_pos_checkbox = tk.Checkbutton(side_panel, text="Force Pos on click?", variable=force_pos_var)
force_pos_checkbox.grid(row=8, column=0, columnspan=2, padx=5, pady=5)

# Add space before the emergency section
side_panel.grid_rowconfigure(9, minsize=50)

# Emergency section
emergency_label = tk.Label(side_panel, text="-------- Emergency --------", font=("Arial", 10, "bold"))
emergency_label.grid(row=10, column=0, columnspan=2, padx=5, pady=5)
force_heal_var = tk.IntVar()
force_heal_checkbox = tk.Checkbutton(side_panel, text="Force Heal?", variable=force_heal_var)
force_heal_checkbox.grid(row=11, column=0, columnspan=2, padx=5, pady=5)

# Emergency buttons
reboot_button = tk.Button(side_panel, text="Reboot recovery sequence", bg="orange", width=25)
hard_reboot_button = tk.Button(side_panel, text="Hard reboot", bg="red", width=25)
reboot_button.grid(row=12, column=0, columnspan=2, padx=5, pady=5)
hard_reboot_button.grid(row=13, column=0, columnspan=2, padx=5, pady=5)

# Placeholder for the image to prevent garbage collection
tk_image = None

# Variables to store the latest image and lock
latest_image = None
image_lock = threading.Lock()

# Function to update the image in the GUI
def update_image():
    global tk_image

    with image_lock:
        if latest_image is not None:
            # Convert the image from BGR to RGB
            combined_image_rgb = cv2.cvtColor(latest_image, cv2.COLOR_BGR2RGB)

            # Convert the image to PIL Image
            pil_image = Image.fromarray(combined_image_rgb)

            # Convert the PIL Image to a PhotoImage
            tk_image = ImageTk.PhotoImage(image=pil_image)

            # Update the image_label
            image_label.config(image=tk_image)
            image_label.image = tk_image  # Keep a reference

    # Schedule the next update
    root.after(20, update_image)

# Mouse event handlers
def on_left_click(event):
    global last_click

    x = event.x
    y = event.y

    total_width = combined_image.shape[1]  # Assuming combined_image is available
    if x < 0 or x > total_width or y < 0 or y > combined_image.shape[0]:
        return

    if x < WINDOW_IMAGE_SIZE:
        # Left image (cropped over opponent)
        pass  # Left clicks on left image are ignored
    else:
        # Right image (full field)
        x_adjusted = x - WINDOW_IMAGE_SIZE
        print("Shifted from: ", x, " to: ", x_adjusted)

        last_click.position_clicked = (x_adjusted, y)
        last_click.data_type = POSITION

def on_right_click(event):
    global last_click

    x = event.x
    y = event.y

    total_width = combined_image.shape[1]  # Assuming combined_image is available
    if x < 0 or x > total_width or y < 0 or y > combined_image.shape[0]:
        return

    if x < WINDOW_IMAGE_SIZE:
        # Right clicks on left image are ignored
        pass
    else:
        # Right image (full field)
        x_adjusted = x - WINDOW_IMAGE_SIZE
        print("Shifted from: ", x, " to: ", x_adjusted)
        last_click.position_clicked = (x_adjusted, y)
        last_click.data_type = OPPONENT_POSITION

def on_mouse_move(event):
    global last_click

    x = event.x
    y = event.y

    total_width = combined_image.shape[1]  # Assuming combined_image is available
    if x < 0 or x > total_width or y < 0 or y > combined_image.shape[0]:
        return

    if x < WINDOW_IMAGE_SIZE:
        # Left image (cropped over opponent)
        last_click.data_type = OPPONENT_ROTATION
        last_click.position_clicked = (x, y)
    else:
        # Right image (full field)
        pass

# Bind the mouse events
image_label.bind("<Button-1>", on_left_click)
image_label.bind("<Button-3>", on_right_click)
image_label.bind("<Motion>", on_mouse_move)

# Thread to receive data from the server
def receive_thread():
    global robotPosX, robotPosY, opponentPosX, opponentPosY, opponentAngleDeg, latest_image, combined_image

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

            def draw_x(img, center, color=(0, 0, 255), size=20, thickness=2):
                x, y = center
                x, y = int(x), int(y)
                cv2.line(img, (x - size, y - size), (x + size, y + size), color, thickness)
                cv2.line(img, (x - size, y + size), (x + size, y - size), color, thickness)

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

        # Update the latest_image
        with image_lock:
            latest_image = combined_image.copy()

# Start the image update loop
update_image()

# Handle window close event to close the socket
def on_closing():
    global client_socket
    if client_socket is not None:
        client_socket.close()
    root.destroy()

root.protocol("WM_DELETE_WINDOW", on_closing)

# Start the Tkinter main loop
root.mainloop()
