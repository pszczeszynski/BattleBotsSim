import socket
import cv2
import numpy as np
import threading
from PIL import Image, ImageTk
import tkinter as tk
from tkinter import ttk, HORIZONTAL
from typing import Tuple, Optional

# Constants
POSITION = 0
OPPONENT_POSITION = 1
OPPONENT_ROTATION = 2
OPPONENT_ROTATION_VEL = 3

WINDOW_IMAGE_SIZE = 720
RAW_IMAGE_SIZE = 360
SCALE_FACTOR = float(WINDOW_IMAGE_SIZE) / RAW_IMAGE_SIZE

MAX_BUFFER_SIZE = 65507  # Maximum UDP payload size


class LastClickData:
    def __init__(self, position: Tuple[int, int] = (0, 0), data_type: int = POSITION):
        self.position: Tuple[int, int] = position
        self.data_type: int = data_type


# Initialize global variables
client_socket: Optional[socket.socket] = None
current_mode: str = "pos_and_rot"  # Default mode
last_click = LastClickData()
combined_image: Optional[np.ndarray] = None
primary_click_action = 'robot'  # Initialize primary click action

# Counters for button presses
auto_l_count = 0
auto_r_count = 0
hard_reboot_count = 0
reboot_recovery_count = 0


# Function to send data to robot controller
def send_data_to_robot_controller() -> None:
    global client_socket
    global auto_l_count, auto_r_count, hard_reboot_count, reboot_recovery_count

    # Collect UI state
    foreground_min_delta_value = int(foreground_min_delta_slider.get())
    background_heal_rate_value = int(background_heal_rate_slider.get())
    force_pos_value = int(force_pos_var.get())
    force_heal_value = int(force_heal_var.get())

    # Prepare data to send
    data = (
        last_click.data_type.to_bytes(4, byteorder="little")
        + int(last_click.position[0] / SCALE_FACTOR).to_bytes(4, byteorder="little")
        + int(last_click.position[1] / SCALE_FACTOR).to_bytes(4, byteorder="little")
        + foreground_min_delta_value.to_bytes(4, byteorder="little")
        + background_heal_rate_value.to_bytes(4, byteorder="little")
        + force_pos_value.to_bytes(4, byteorder="little")
        + force_heal_value.to_bytes(4, byteorder="little")
        + auto_l_count.to_bytes(4, byteorder="little")
        + auto_r_count.to_bytes(4, byteorder="little")
        + hard_reboot_count.to_bytes(4, byteorder="little")
        + reboot_recovery_count.to_bytes(4, byteorder="little")
    )

    # print("Sending click data:", last_click.data_type, last_click.position)
    # print("Auto L count:", auto_l_count, "Auto R count:", auto_r_count)
    # print("foreground_min_delta:", foreground_min_delta_value, "background_heal_rate:", background_heal_rate_value)
    # print("Hard reboot count:", hard_reboot_count, "Reboot recovery count:", reboot_recovery_count)

    if client_socket is not None:
        client_socket.sendto(data, (HOST, PORT))
    else:
        print("Not connected yet.")


# Function to parse server data
def parse_server_data(
    data: bytes,
) -> Optional[Tuple[int, int, int, int, int, np.ndarray]]:
    curr_pos = 0
    try:
        robot_pos_x = int.from_bytes(data[curr_pos : curr_pos + 4], byteorder="big")
        curr_pos += 4
        robot_pos_y = int.from_bytes(data[curr_pos : curr_pos + 4], byteorder="big")
        curr_pos += 4
        opponent_pos_x = int.from_bytes(data[curr_pos : curr_pos + 4], byteorder="big")
        curr_pos += 4
        opponent_pos_y = int.from_bytes(data[curr_pos : curr_pos + 4], byteorder="big")
        curr_pos += 4
        opponent_angle_deg = int.from_bytes(
            data[curr_pos : curr_pos + 4], byteorder="big"
        )
        curr_pos += 4
        image_size = int.from_bytes(data[curr_pos : curr_pos + 4], byteorder="big")
        curr_pos += 4
        image_data = data[curr_pos:]

        if len(image_data) < image_size:
            print("Error: Incomplete image data received")
            return None

        # Convert the received data to a numpy array
        image_array = np.frombuffer(image_data[:image_size], dtype=np.uint8)
        return (
            robot_pos_x,
            robot_pos_y,
            opponent_pos_x,
            opponent_pos_y,
            opponent_angle_deg,
            image_array,
        )
    except Exception as e:
        print("Error parsing server data:", e)
        return None


# Functions for drawing on images
def draw_robot_position(
    image: np.ndarray,
    position: Tuple[int, int],
    color: Tuple[int, int, int] = (0, 255, 0),
    radius: int = 10,
) -> None:
    # sanity check
    if position[0] < 0 or position[1] < 0 or position[0] >= image.shape[1] or position[1] >= image.shape[0]:
        return

    cv2.circle(image, position, radius=radius, color=color, thickness=2)


def draw_opponent_position(
    image: np.ndarray,
    position: Tuple[int, int],
    color: Tuple[int, int, int] = (255, 0, 0),
    radius: int = 10,
) -> None:
    # sanity check
    if position[0] < 0 or position[1] < 0 or position[0] >= image.shape[1] or position[1] >= image.shape[0]:
        return

    cv2.circle(image, position, radius=radius, color=color, thickness=2)


def draw_opponent_rotation(
    image: np.ndarray,
    center: Tuple[int, int],
    angle_deg: float,
    length: int = 50,
    color: Tuple[int, int, int] = (0, 255, 0),
) -> None:
    angle_rad = np.radians(angle_deg)
    x_arrow = int(center[0] + length * np.cos(angle_rad))
    y_arrow = int(center[1] + length * np.sin(angle_rad))
    cv2.arrowedLine(image, center, (x_arrow, y_arrow), color=color, thickness=2)


def draw_last_click_marker(image: np.ndarray, last_click: LastClickData) -> None:
    if last_click.data_type == POSITION:
        x_draw, y_draw = map(int, last_click.position)
        draw_x_marker(image, (x_draw, y_draw), color=(0, 0, 255))
    elif last_click.data_type == OPPONENT_POSITION:
        x_draw, y_draw = map(int, last_click.position)
        cv2.circle(image, (x_draw, y_draw), radius=20, color=(255, 0, 0), thickness=2)


def draw_x_marker(
    image: np.ndarray,
    center: Tuple[int, int],
    color: Tuple[int, int, int] = (0, 0, 255),
    size: int = 10,
    thickness: int = 2,
) -> None:
    x, y = center
    cv2.line(image, (x - size, y - size), (x + size, y + size), color, thickness)
    cv2.line(image, (x - size, y + size), (x + size, y - size), color, thickness)


# Function to crop image around a point
def crop_image_around_point(
    image: np.ndarray, center_pos: Tuple[int, int], crop_size: int
) -> np.ndarray:
    x = center_pos[0] - crop_size // 2
    y = center_pos[1] - crop_size // 2

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
        mode="constant",
        constant_values=0,
    )

    # Adjust coordinates due to padding
    x1_new = x1 + pad_left
    y1_new = y1 + pad_top

    # Crop the image
    cropped_image = image_padded[
        y1_new : y1_new + crop_size, x1_new : x1_new + crop_size
    ]

    return cropped_image


# Function to process 'pos_only' mode
def process_pos_only_mode(
    received_image: np.ndarray,
    robot_pos: Tuple[int, int],
    opponent_pos: Tuple[int, int],
    last_click: LastClickData,
) -> np.ndarray:
    full_field_image = cv2.resize(
        received_image.copy(), (WINDOW_IMAGE_SIZE, WINDOW_IMAGE_SIZE)
    )

    original_height, original_width = received_image.shape[:2]
    scale_factor_x = original_width / WINDOW_IMAGE_SIZE
    scale_factor_y = original_height / WINDOW_IMAGE_SIZE

    screen_robot_pos = (
        int(robot_pos[0] / scale_factor_x),
        int(robot_pos[1] / scale_factor_y),
    )
    screen_opponent_pos = (
        int(opponent_pos[0] / scale_factor_x),
        int(opponent_pos[1] / scale_factor_y),
    )

    draw_robot_position(full_field_image, screen_robot_pos)
    draw_opponent_position(full_field_image, screen_opponent_pos)

    # draw_last_click_marker(full_field_image, last_click)

    return full_field_image


# Function to process 'pos_and_rot' mode
def process_pos_and_rot_mode(
    received_image: np.ndarray,
    robot_pos: Tuple[int, int],
    opponent_pos: Tuple[int, int],
    opponent_angle_deg: float,
    last_click: LastClickData,
) -> np.ndarray:
    left_image = crop_image_around_point(received_image, opponent_pos, RAW_IMAGE_SIZE)
    left_image = cv2.resize(left_image, (WINDOW_IMAGE_SIZE, WINDOW_IMAGE_SIZE))

    right_image = cv2.resize(
        received_image.copy(), (WINDOW_IMAGE_SIZE, WINDOW_IMAGE_SIZE)
    )

    combined_image = np.hstack((left_image, right_image))

    rel_robot_pos_x = robot_pos[0] - opponent_pos[0]
    rel_robot_pos_y = robot_pos[1] - opponent_pos[1]

    scale_factor_left = WINDOW_IMAGE_SIZE / RAW_IMAGE_SIZE

    screen_robot_pos_left = (
        int((rel_robot_pos_x + RAW_IMAGE_SIZE // 2) * scale_factor_left),
        int((rel_robot_pos_y + RAW_IMAGE_SIZE // 2) * scale_factor_left),
    )
    screen_opponent_pos_left = (WINDOW_IMAGE_SIZE // 2, WINDOW_IMAGE_SIZE // 2)

    draw_robot_position(combined_image[:, :WINDOW_IMAGE_SIZE], screen_robot_pos_left)
    draw_opponent_position(
        combined_image[:, :WINDOW_IMAGE_SIZE], screen_opponent_pos_left
    )
    draw_opponent_rotation(
        combined_image[:, :WINDOW_IMAGE_SIZE],
        screen_opponent_pos_left,
        opponent_angle_deg,
    )

    original_height, original_width = received_image.shape[:2]
    scale_factor_right_x = original_width / WINDOW_IMAGE_SIZE
    scale_factor_right_y = original_height / WINDOW_IMAGE_SIZE

    screen_robot_pos_right = (
        int(robot_pos[0] / scale_factor_right_x),
        int(robot_pos[1] / scale_factor_right_y),
    )
    screen_opponent_pos_right = (
        int(opponent_pos[0] / scale_factor_right_x),
        int(opponent_pos[1] / scale_factor_right_y),
    )

    draw_robot_position(combined_image[:, WINDOW_IMAGE_SIZE:], screen_robot_pos_right)
    draw_opponent_position(
        combined_image[:, WINDOW_IMAGE_SIZE:], screen_opponent_pos_right
    )
    draw_opponent_rotation(
        combined_image[:, WINDOW_IMAGE_SIZE:],
        screen_opponent_pos_right,
        opponent_angle_deg,
    )

    if last_click.data_type == POSITION or last_click.data_type == OPPONENT_POSITION:
        x_draw, y_draw = map(int, last_click.position)
        if last_click.data_type == POSITION:
            draw_x_marker(combined_image[:, WINDOW_IMAGE_SIZE:], (x_draw, y_draw))
        else:
            cv2.circle(
                combined_image[:, WINDOW_IMAGE_SIZE:],
                (x_draw, y_draw),
                radius=10,
                color=(255, 0, 0),
                thickness=2,
            )
    elif last_click.data_type == OPPONENT_ROTATION:
        x_draw, y_draw = map(int, last_click.position)
        center_left = (WINDOW_IMAGE_SIZE // 2, WINDOW_IMAGE_SIZE // 2)
        cv2.arrowedLine(
            combined_image[:, :WINDOW_IMAGE_SIZE],
            center_left,
            (x_draw, y_draw),
            color=(0, 255, 0),
            thickness=2,
        )

    return combined_image


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
ip_entry.insert(0, "192.168.0.115")  # Default value
ip_entry.grid(row=0, column=1, padx=5, pady=5)

port_label = tk.Label(side_panel, text="Port:")
port_label.grid(row=1, column=0, padx=5, pady=5)
port_entry = tk.Entry(side_panel)
port_entry.insert(0, "11118")  # Default value
port_entry.grid(row=1, column=1, padx=5, pady=5)


# Connect button
def connect_to_server() -> None:
    global HOST, PORT, client_socket

    HOST = ip_entry.get()
    PORT = int(port_entry.get())

    # Create a UDP socket
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print("UDP socket created.")

    # Send a test message
    client_socket.sendto(b"Hello", (HOST, PORT))
    print("Test message sent.")

    # Start the receive thread
    threading.Thread(target=receive_thread, daemon=True).start()


connect_button = tk.Button(side_panel, text="Connect", command=connect_to_server)
connect_button.grid(row=2, column=0, columnspan=2, padx=5, pady=5)

# Mode selection
mode_label = tk.Label(side_panel, text="Mode:")
mode_label.grid(row=3, column=0, columnspan=2, padx=5, pady=5)

current_mode_var = tk.StringVar(value="pos_and_rot")  # Default mode


def on_mode_change(*args) -> None:
    global current_mode
    current_mode = current_mode_var.get()
    print(f"Mode set to: {current_mode}")


current_mode_var.trace("w", on_mode_change)

pos_only_radiobutton = tk.Radiobutton(
    side_panel, text="Pos Only", variable=current_mode_var, value="pos_only"
)
pos_and_rot_radiobutton = tk.Radiobutton(
    side_panel, text="Pos and Rot", variable=current_mode_var, value="pos_and_rot"
)

pos_only_radiobutton.grid(row=4, column=0, columnspan=2, padx=5, pady=5)
pos_and_rot_radiobutton.grid(row=5, column=0, columnspan=2, padx=5, pady=5)

# Auto L and Auto R buttons
def on_auto_l_button_press():
    global auto_l_count
    auto_l_count += 1
    send_data_to_robot_controller()

def on_auto_r_button_press():
    global auto_r_count
    auto_r_count += 1
    send_data_to_robot_controller()

def on_foreground_min_delta_change(event):
    send_data_to_robot_controller()

def on_background_heal_rate_change(event):
    send_data_to_robot_controller()

def on_force_pos_change():
    send_data_to_robot_controller()


auto_l_button = tk.Button(side_panel, text="Auto L", bg="lightblue", width=10, command=on_auto_l_button_press)
auto_r_button = tk.Button(side_panel, text="Auto R", bg="red", width=10, command=on_auto_r_button_press)
auto_l_button.grid(row=6, column=0, padx=5, pady=5)
auto_r_button.grid(row=6, column=1, padx=5, pady=5)

# Sliders with values displayed
foreground_label = tk.Label(side_panel, text="Foreground min delta:")
foreground_label.grid(row=7, column=0, columnspan=2, padx=5, pady=5)

foreground_min_delta_slider = ttk.Scale(side_panel, from_=1, to=30, orient=HORIZONTAL, command=on_foreground_min_delta_change)
foreground_min_delta_slider.grid(row=8, column=0, columnspan=2, padx=5, pady=5)

background_label = tk.Label(side_panel, text="Background Heal Rate:")
background_label.grid(row=9, column=0, columnspan=2, padx=5, pady=5)

background_heal_rate_slider = ttk.Scale(side_panel, from_=1, to=300, orient=HORIZONTAL, command=on_background_heal_rate_change)
background_heal_rate_slider.grid(row=10, column=0, columnspan=2, padx=5, pady=5)

# Checkboxes
force_pos_var = tk.IntVar()
force_pos_checkbox = tk.Checkbutton(
    side_panel, text="Force Pos on click?", variable=force_pos_var, command=on_force_pos_change
)
force_pos_checkbox.grid(row=11, column=0, columnspan=2, padx=5, pady=5)

force_heal_var = tk.IntVar()
force_heal_checkbox = tk.Checkbutton(
    side_panel, text="Force Heal?", variable=force_heal_var
)
force_heal_checkbox.grid(row=12, column=0, columnspan=2, padx=5, pady=5)

# Emergency buttons
def on_reboot_recovery_button_press():
    global reboot_recovery_count
    reboot_recovery_count += 1
    send_data_to_robot_controller()

def on_hard_reboot_button_press():
    global hard_reboot_count
    hard_reboot_count += 1
    send_data_to_robot_controller()

# reboot_button = tk.Button(
#     side_panel, text="Reboot recovery sequence", bg="orange", width=25, command=on_reboot_recovery_button_press
# )
hard_reboot_button = tk.Button(side_panel, text="Hard reboot", bg="red", width=25, command=on_hard_reboot_button_press)
# reboot_button.grid(row=13, column=0, columnspan=2, padx=5, pady=5)
hard_reboot_button.grid(row=14, column=0, columnspan=2, padx=5, pady=5)

# Add a big button to toggle the primary click action
def toggle_primary_click_action():
    global primary_click_action, primary_click_button
    if primary_click_action == 'robot':
        primary_click_action = 'opponent'
        primary_click_button.config(text="Primary: Opponent")
    else:
        primary_click_action = 'robot'
        primary_click_button.config(text="Primary: Robot")

primary_click_button = tk.Button(side_panel, text="Primary: Robot", command=toggle_primary_click_action, width=20, height=2)
primary_click_button.grid(row=15, column=0, columnspan=2, padx=5, pady=10)

# Placeholder for the image to prevent garbage collection
tk_image: Optional[ImageTk.PhotoImage] = None

# Variables to store the latest image and lock
latest_image: Optional[np.ndarray] = None
image_lock = threading.Lock()


# Function to update the image in the GUI
def update_image() -> None:
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
def on_left_click(event: tk.Event) -> None:
    if primary_click_action == 'robot':
        handle_mouse_event(event, POSITION)
    else:
        handle_mouse_event(event, OPPONENT_POSITION)


def on_right_click(event: tk.Event) -> None:
    if primary_click_action == 'robot':
        handle_mouse_event(event, OPPONENT_POSITION)
    else:
        handle_mouse_event(event, POSITION)

def on_left_drag(event: tk.Event) -> None:
    if current_mode == "pos_and_rot" and event.x < WINDOW_IMAGE_SIZE:
        handle_mouse_event(event, OPPONENT_ROTATION)

    if primary_click_action == 'robot':
        handle_mouse_event(event, POSITION)
    else:
        handle_mouse_event(event, OPPONENT_POSITION)


def on_right_drag(event: tk.Event) -> None:
    if primary_click_action == 'robot':
        handle_mouse_event(event, OPPONENT_POSITION)
    else:
        handle_mouse_event(event, POSITION)


def on_mouse_move(event: tk.Event) -> None:
    if current_mode == "pos_and_rot" and event.x < WINDOW_IMAGE_SIZE:
        handle_mouse_event(event, OPPONENT_ROTATION)
    
    elif primary_click_action == 'robot':
        handle_mouse_event(event, POSITION)
    else:
        handle_mouse_event(event, OPPONENT_POSITION)


def handle_mouse_event(event: tk.Event, data_type: int) -> None:
    global last_click, combined_image

    if combined_image is None:
        return

    x, y = event.x, event.y

    if current_mode == "pos_and_rot":
        total_width = combined_image.shape[1]
        if x < 0 or x > total_width or y < 0 or y > combined_image.shape[0]:
            return

        if data_type == POSITION:
            if x >= WINDOW_IMAGE_SIZE:
                x_adjusted = x - WINDOW_IMAGE_SIZE
                last_click.position = (x_adjusted, y)
                last_click.data_type = POSITION
        elif data_type == OPPONENT_POSITION:
            if x >= WINDOW_IMAGE_SIZE:
                x_adjusted = x - WINDOW_IMAGE_SIZE
                last_click.position = (x_adjusted, y)
                last_click.data_type = OPPONENT_POSITION
        elif data_type == OPPONENT_ROTATION:
            if x < WINDOW_IMAGE_SIZE:
                last_click.position = (x, y)
                last_click.data_type = OPPONENT_ROTATION
    elif current_mode == "pos_only":
        if data_type == POSITION or data_type == OPPONENT_POSITION:
            last_click.position = (x, y)
            last_click.data_type = data_type

    send_data_to_robot_controller()


# Bind the mouse events
image_label.bind("<ButtonPress-1>", on_left_click)
image_label.bind("<ButtonPress-3>", on_right_click)
image_label.bind("<B1-Motion>", on_left_drag)
image_label.bind("<B3-Motion>", on_right_drag)
image_label.bind("<Motion>", on_mouse_move)


# Thread to receive data from the server
def receive_thread() -> None:
    global latest_image, combined_image

    while True:
        try:
            data, _ = client_socket.recvfrom(MAX_BUFFER_SIZE)
        except Exception as e:
            print("Error receiving data:", e)
            send_data_to_robot_controller()
            continue

        if len(data) < 4:
            print("Error: Incomplete size data received")
            send_data_to_robot_controller()
            continue

        parsed_data = parse_server_data(data)
        if parsed_data is None:
            continue

        (
            robot_pos_x,
            robot_pos_y,
            opponent_pos_x,
            opponent_pos_y,
            opponent_angle_deg,
            image_array,
        ) = parsed_data
        received_image = cv2.imdecode(image_array, cv2.IMREAD_COLOR)

        robot_pos = (robot_pos_x, robot_pos_y)
        opponent_pos = (opponent_pos_x, opponent_pos_y)

        if current_mode == "pos_only":
            combined_image = process_pos_only_mode(
                received_image, robot_pos, opponent_pos, last_click
            )
        elif current_mode == "pos_and_rot":
            combined_image = process_pos_and_rot_mode(
                received_image, robot_pos, opponent_pos, opponent_angle_deg, last_click
            )

        # Update the latest_image
        with image_lock:
            latest_image = combined_image.copy()
        
        send_data_to_robot_controller()



# Start the image update loop
update_image()


# Handle window close event to close the socket
def on_closing() -> None:
    global client_socket
    if client_socket is not None:
        client_socket.close()
    root.destroy()


root.protocol("WM_DELETE_WINDOW", on_closing)

# Start the Tkinter main loop
root.mainloop()
