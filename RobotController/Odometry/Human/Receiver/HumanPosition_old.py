import socket
import time
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

WINDOW_IMAGE_SIZE = 360
RAW_IMAGE_SIZE = 360
SCALE_FACTOR = float(WINDOW_IMAGE_SIZE) / RAW_IMAGE_SIZE

MAX_BUFFER_SIZE = 65507  # Maximum UDP payload size

# Display scaling factor
DISPLAY_SCALE_FACTOR = 2  # Multiply the display size by 2

HOST = "192.168.8.158"
PORT = 11118
# Counters for button presses
auto_l_count = 0
auto_r_count = 0
hard_reboot_count = 0
reboot_recovery_count = 0
load_start_count = 0
load_saved_count = 0
save_count = 0

class LastClickData:
    def __init__(self, position: Tuple[int, int] = (0, 0), data_type: int = POSITION):
        self.position: Tuple[int, int] = position
        self.data_type: int = data_type

# Initialize global variables
root = tk.Tk() # Main TK root
client_socket: Optional[socket.socket] = None
current_mode: str = "pos_and_rot"  # Default mode
last_click = LastClickData()
combined_image: Optional[np.ndarray] = None
primary_click_action = 'robot'  # Initialize primary click action
lastSliderTime = time.time()-2
ignore_callbacks = False
force_pos_var = tk.IntVar()
force_heal_var = tk.IntVar()

# Function to send data to robot controller
def send_data_to_robot_controller() -> None:
    global client_socket
    global auto_l_count, auto_r_count, hard_reboot_count, reboot_recovery_count
    global force_pos_var, force_heal_var, lastSliderTime

    try:
        # Collect UI state
        foreground_min_delta_value = int(fg_min_delta.get())
        background_heal_rate_value = int(bg_heal_rate.get())
        force_pos_value = int(force_pos_var.get())
        force_heal_value = int(force_heal_var.get())
        new_slider_data = int((time.time() - lastSliderTime) < 0.5)
        password = 11115

        # Prepare data to send
        data = (
            password.to_bytes(4, byteorder="little")
            + last_click.data_type.to_bytes(4, byteorder="little")
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
            + load_start_count.to_bytes(4, byteorder="little")
            + load_saved_count.to_bytes(4, byteorder="little")
            + save_count.to_bytes(4, byteorder="little")
            + new_slider_data.to_bytes(4, byteorder="little")
        )

        if client_socket is not None:
            client_socket.sendto(data, (HOST, PORT))
        else:
            print("Not connected yet.")
    except Exception as e:
        print("Error sending data to robot", e)


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
    global ignore_callbacks, lastSliderTime
    if ignore_callbacks:
        return
    try:
        fg_min_delta.set(str(int(foreground_min_delta_slider.get())))
    except Exception as e:
        print("Error converting slider min delta")
        return None

    lastSliderTime = time.time()
    send_data_to_robot_controller()

def on_background_heal_rate_change(event):
    global ignore_callbacks, lastSliderTime
    if ignore_callbacks:
        return

    lastSliderTime = time.time()
    try:
        bg_heal_rate.set(str(int(background_heal_rate_slider.get())))
    except Exception as e:
        print("Error converting background heal rate")
        return None

    send_data_to_robot_controller()

def on_force_pos_change():
    send_data_to_robot_controller()

# Create the main GUI window
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

status_label = tk.StringVar()
status_label.set("Not Connected")

fg_min_delta = tk.StringVar()
fg_min_delta.set("5")

bg_heal_rate = tk.StringVar()
bg_heal_rate.set("50")

foreground_min_delta_slider = ttk.Scale(side_panel, from_=1, to=30, orient=HORIZONTAL, length=250, command=on_foreground_min_delta_change)
foreground_min_delta_slider.set(5)





# Function to parse server data
def parse_server_data(
    data: bytes,
) -> Optional[Tuple[int, int, int, int, int, np.ndarray, dict]]:
    curr_pos = 0
    try:
        # Find the end of the text data. Assuming '$' is the separator
        end_text = data.index(b'$')  # Find our binary data separator
        text_data = data[:end_text].decode('utf-8', errors='ignore')
        image_data = data[end_text + 1:]

        # Split the string data by '%'
        parts = text_data.split('%')  # Split at most 6 times to keep the rest as image data

        # Extract numeric data from strings
        if len(parts) < 9:
            print("Error: Not enough data parts to parse")
            return None

        index = 0
        password = int(parts[index])
        if password != 11115:
            return None
        index += 1
        robot_pos_x = int(parts[index])
        index += 1
        robot_pos_y = int(parts[index])
        index += 1
        opponent_pos_x = int(parts[index])
        index += 1
        opponent_pos_y = int(parts[index])
        index += 1
        opponent_angle_deg = int(parts[index])
        index += 1
        fg_min_ratio_int = int(parts[index])
        index += 1
        bg_heal_rate_int = int(parts[index])
        index += 1

        # Collect GUI updates
        gui_updates = {}
        if (time.time() - lastSliderTime) > 0.5:
            gui_updates['fg_min_delta'] = str(fg_min_ratio_int)
            gui_updates['foreground_min_delta_slider'] = fg_min_ratio_int
            gui_updates['bg_heal_rate'] = str(bg_heal_rate_int)
            gui_updates['background_heal_rate_slider'] = bg_heal_rate_int
            gui_updates['force_heal_var'] = int(parts[index])


        index += 1
        image_size = int(parts[index])

        # Convert image data string to bytes
        # Check if image_data is valid
        if len(image_data) < image_size:
            print("Error: Incomplete or no image data received")
            # return None

        # Convert the received image data to a numpy array
        image_array = np.frombuffer(image_data, dtype=np.uint8)

        return (
            robot_pos_x,
            robot_pos_y,
            opponent_pos_x,
            opponent_pos_y,
            opponent_angle_deg,
            image_array,
            gui_updates,
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
    # Sanity check
    if position[0] < 0 or position[1] < 0 or position[0] >= image.shape[1] or position[1] >= image.shape[0]:
        return

    try:
        cv2.circle(image, position, radius=radius, color=color, thickness=2)
    except Exception as e:
        print("Error drawing circle", e)
        return None

def draw_opponent_position(
    image: np.ndarray,
    position: Tuple[int, int],
    color: Tuple[int, int, int] = (255, 0, 0),
    radius: int = 10,
) -> None:
    # Sanity check
    if position[0] < 0 or position[1] < 0 or position[0] >= image.shape[1] or position[1] >= image.shape[0]:
        return

    try:
        cv2.circle(image, position, radius=radius, color=color, thickness=2)
    except Exception as e:
        print("Error drawing circle", e)
        return

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
    try:
        if last_click.data_type == POSITION:
            x_draw, y_draw = map(int, last_click.position)
            draw_x_marker(image, (x_draw, y_draw), color=(0, 0, 255))
        elif last_click.data_type == OPPONENT_POSITION:
            x_draw, y_draw = map(int, last_click.position)
            cv2.circle(image, (x_draw, y_draw), radius=20, color=(255, 0, 0), thickness=2)
    except Exception as e:
        print("Error drawing click marker", e)
        return

def draw_x_marker(
    image: np.ndarray,
    center: Tuple[int, int],
    color: Tuple[int, int, int] = (0, 0, 255),
    size: int = 10,
    thickness: int = 2,
) -> None:
    try:
        x, y = center
        cv2.line(image, (x - size, y - size), (x + size, y + size), color, thickness)
        cv2.line(image, (x - size, y + size), (x + size, y - size), color, thickness)
    except Exception as e:
        print("Error drawing x marker", e)
        return

# Function to crop image around a point
def crop_image_around_point(
    image: np.ndarray, center_pos: Tuple[int, int], crop_size: int
) -> np.ndarray:
    try:
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

    except Exception as e:
        print("Error cropping image", e)
        return image

# Function to process 'pos_only' mode
def process_pos_only_mode(
    received_image: np.ndarray,
    robot_pos: Tuple[int, int],
    opponent_pos: Tuple[int, int],
    last_click: LastClickData,
) -> np.ndarray:

    full_field_image = received_image.copy()  # No resizing
    try:
        screen_robot_pos = robot_pos
        screen_opponent_pos = opponent_pos

        draw_robot_position(full_field_image, screen_robot_pos)
        draw_opponent_position(full_field_image, screen_opponent_pos)

    except Exception as e:
        print("Error pos only mode", e)

    return full_field_image

# Function to process 'pos_and_rot' mode
def process_pos_and_rot_mode(
    received_image: np.ndarray,
    robot_pos: Tuple[int, int],
    opponent_pos: Tuple[int, int],
    opponent_angle_deg: float,
    last_click: LastClickData,
) -> np.ndarray:
    try:
        left_image = crop_image_around_point(received_image, opponent_pos, RAW_IMAGE_SIZE)
        right_image = received_image.copy()

        # Ensure both images have the same height
        if left_image.shape[0] != right_image.shape[0]:
            desired_height = right_image.shape[0]
            left_image = cv2.resize(left_image, (left_image.shape[1], desired_height))  # Only downscaling if needed

        combined_image = np.hstack((left_image, right_image))

        # Adjust positions accordingly
        rel_robot_pos_x = robot_pos[0] - opponent_pos[0]
        rel_robot_pos_y = robot_pos[1] - opponent_pos[1]

        screen_robot_pos_left = (
            int(rel_robot_pos_x + left_image.shape[1] // 2),
            int(rel_robot_pos_y + left_image.shape[0] // 2),
        )
        screen_opponent_pos_left = (left_image.shape[1] // 2, left_image.shape[0] // 2)

        draw_robot_position(combined_image[:, :left_image.shape[1]], screen_robot_pos_left)
        draw_opponent_position(
            combined_image[:, :left_image.shape[1]], screen_opponent_pos_left
        )
        draw_opponent_rotation(
            combined_image[:, :left_image.shape[1]],
            screen_opponent_pos_left,
            opponent_angle_deg,
        )

        # For right_image
        screen_robot_pos_right = robot_pos
        screen_opponent_pos_right = opponent_pos

        draw_robot_position(combined_image[:, left_image.shape[1]:], screen_robot_pos_right)
        draw_opponent_position(
            combined_image[:, left_image.shape[1]:], screen_opponent_pos_right
        )
        draw_opponent_rotation(
            combined_image[:, left_image.shape[1]:],
            screen_opponent_pos_right,
            opponent_angle_deg,
        )

        # Handle last click events (adjust if necessary)
        if last_click.data_type == POSITION or last_click.data_type == OPPONENT_POSITION:
            x_draw, y_draw = map(int, last_click.position)
            if last_click.data_type == POSITION:
                draw_x_marker(combined_image[:, left_image.shape[1]:], (x_draw, y_draw))
            else:
                cv2.circle(
                    combined_image[:, left_image.shape[1]:],
                    (x_draw, y_draw),
                    radius=10,
                    color=(255, 0, 0),
                    thickness=2,
                )
        elif last_click.data_type == OPPONENT_ROTATION:
            x_draw, y_draw = map(int, last_click.position)
            center_left = (left_image.shape[1] // 2, left_image.shape[0] // 2)
            cv2.arrowedLine(
                combined_image[:, :left_image.shape[1]],
                center_left,
                (x_draw, y_draw),
                color=(0, 255, 0),
                thickness=2,
            )

        return combined_image

    except Exception as e:
        print("Error process pos and rot mode", e)
        return received_image.copy()

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
    with image_lock:
        if current_mode == "pos_and_rot" and event.x < (latest_image.shape[1] * DISPLAY_SCALE_FACTOR) // 2:
            handle_mouse_event(event, OPPONENT_ROTATION)
        elif primary_click_action == 'robot':
            handle_mouse_event(event, POSITION)
        else:
            handle_mouse_event(event, OPPONENT_POSITION)

def on_right_drag(event: tk.Event) -> None:
    if primary_click_action == 'robot':
        handle_mouse_event(event, OPPONENT_POSITION)
    else:
        handle_mouse_event(event, POSITION)

def on_mouse_move(event: tk.Event) -> None:
    with image_lock:
        if current_mode == "pos_and_rot" and event.x < (latest_image.shape[1] * DISPLAY_SCALE_FACTOR) // 2:
            handle_mouse_event(event, OPPONENT_ROTATION)
        elif primary_click_action == 'robot':
            handle_mouse_event(event, POSITION)
        else:
            handle_mouse_event(event, OPPONENT_POSITION)

# IP address and port input fields
currRow = 0

ip_label = tk.Label(side_panel, text="IP Address:")
ip_label.grid(row=currRow, column=0, padx=5, pady=5)
ip_entry = tk.Entry(side_panel)
ip_entry.insert(0, HOST)  # Default value
ip_entry.grid(row=currRow, column=1, padx=5, pady=5)
currRow += 1

port_label = tk.Label(side_panel, text="Port:")
port_label.grid(row=currRow, column=0, padx=5, pady=5)
port_entry = tk.Entry(side_panel)
port_entry.insert(0, PORT)  # Default value
port_entry.grid(row=currRow, column=1, padx=5, pady=5)
currRow += 1

# Connect button
def connect_to_server() -> None:
    global HOST, PORT, client_socket

    HOST = ip_entry.get()
    PORT = int(port_entry.get())

    # Create a UDP socket
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client_socket.settimeout(0.2)
    print("UDP socket created.")

    # Send a test message
    client_socket.sendto(b"Hello", (HOST, PORT))
    print("Test message sent.")

    # Start the receive thread
    threading.Thread(target=receive_thread, daemon=True).start()

connect_button = tk.Button(side_panel, text="Connect", command=connect_to_server)
connect_button.grid(row=currRow, column=0, columnspan=2, padx=5, pady=5)
currRow += 1

# Mode selection
current_mode_var = tk.StringVar(value="pos_and_rot")  # Default mode
if PORT == 11119:
    current_mode_var = tk.StringVar(value="pos_only")
    current_mode = current_mode_var.get()

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

pos_only_radiobutton.grid(row=currRow, column=0, columnspan=1, padx=5, pady=5)
pos_and_rot_radiobutton.grid(row=currRow, column=1, columnspan=1, padx=5, pady=5)
currRow += 1

auto_l_button = tk.Button(side_panel, text="Auto L", bg="lightblue", width=10, command=on_auto_l_button_press)
auto_r_button = tk.Button(side_panel, text="Auto R", bg="red", width=10, command=on_auto_r_button_press)
auto_l_button.grid(row=currRow, column=0, padx=5, pady=5)
auto_r_button.grid(row=currRow, column=1, padx=5, pady=5)
currRow += 1

def on_load_start_button_press():
    global load_start_count
    load_start_count += 1
    send_data_to_robot_controller()

def on_load_saved_button_press():
    global load_saved_count
    load_saved_count += 1
    send_data_to_robot_controller()

def on_hard_reboot_button_press():
    global hard_reboot_count
    hard_reboot_count += 1
    send_data_to_robot_controller()

def on_save_button_press():
    global save_count
    save_count += 1
    send_data_to_robot_controller()

def on_force_heal_button_press():
    global ignore_callbacks
    if ignore_callbacks:
        return
    global lastSliderTime
    lastSliderTime = time.time()
    send_data_to_robot_controller()

load_start_bg = tk.Button(side_panel, text="Load Start", width=10, command=on_load_start_button_press)
load_saved_bg = tk.Button(side_panel, text="Load Saved", width=10, command=on_load_saved_button_press)
load_start_bg.grid(row=currRow, column=0, padx=5, pady=5)
load_saved_bg.grid(row=currRow, column=1, padx=5, pady=5)
currRow += 1

hard_reboot_button = tk.Button(side_panel, text="Reboot", width=10, command=on_hard_reboot_button_press)
save_bg_button = tk.Button(side_panel, text="Save", width=10, command=on_hard_reboot_button_press)
hard_reboot_button.grid(row=currRow, column=0, columnspan=1, padx=5, pady=5)
save_bg_button.grid(row=currRow, column=1, columnspan=1, padx=5, pady=5)
currRow += 1

# Sliders with values displayed
frame_textfg = tk.Frame(side_panel)
frame_textfg.grid(row=currRow, column=0, columnspan=1, padx=5, pady=5)
foreground_label = tk.Label(frame_textfg, text="FG delta: ")
foreground_label.pack(side="left")
foreground_value = tk.Label(frame_textfg, textvariable=fg_min_delta)
foreground_value.pack(side="left")
foreground_min_delta_slider.grid(row=currRow, column=1, columnspan=1, padx=1, pady=5)
currRow += 1

# Sliders with values displayed
frame_textbg = tk.Frame(side_panel)
frame_textbg.grid(row=currRow, column=0, columnspan=1, padx=5, pady=5)
background_label = tk.Label(frame_textbg, text="OBJ heal: ")
background_label.pack(side="left")
background_value = tk.Label(frame_textbg, textvariable=bg_heal_rate)
background_value.pack(side="left")
background_heal_rate_slider = ttk.Scale(side_panel, from_=1, to=400, length=250, orient=HORIZONTAL, command=on_background_heal_rate_change)
background_heal_rate_slider.grid(row=currRow, column=1, columnspan=1, padx=1, pady=5)

currRow += 1
currRow += 1

# Checkboxes
force_pos_checkbox = tk.Checkbutton(
    side_panel, text="Force Pos?", variable=force_pos_var, command=on_force_pos_change
)
force_pos_checkbox.grid(row=currRow, column=1, columnspan=1, padx=5, pady=5)

force_heal_checkbox = tk.Checkbutton(
    side_panel, text="Force Heal?", variable=force_heal_var, command=on_force_heal_button_press)
force_heal_checkbox.grid(row=currRow, column=0, columnspan=1, padx=5, pady=5)
currRow += 1

# Emergency buttons
def on_reboot_recovery_button_press():
    global reboot_recovery_count
    reboot_recovery_count += 1
    send_data_to_robot_controller()

def toggle_primary_robot_action():
    global primary_click_action, primary_click_robot_button, primary_click_opp_button
    primary_click_action = 'robot'
    primary_click_robot_button.config(bg="green")
    primary_click_opp_button.config(bg="grey")

def toggle_primary_opponent_action():
    global primary_click_action, primary_click_robot_button, primary_click_opp_button
    primary_click_action = 'opponent'
    primary_click_robot_button.config(bg="grey")
    primary_click_opp_button.config(bg="red")

if PORT == 11119:
    primary_click_robot_button = tk.Button(side_panel, text="Robot", command=toggle_primary_robot_action, width=15, height=2)
    primary_click_robot_button.grid(row=currRow, column=0, columnspan=1, padx=5, pady=10)

    primary_click_opp_button = tk.Button(side_panel, text="Opponent", command=toggle_primary_opponent_action, width=15, height=2)
    primary_click_opp_button.grid(row=currRow, column=1, columnspan=1, padx=5, pady=10)
else:
    button_panel = tk.Frame(main_panel)
    button_panel.grid(row=1, column=0)

    primary_click_robot_button = tk.Button(button_panel, text="Robot", command=toggle_primary_robot_action, width=15, height=2)
    primary_click_robot_button.grid(row=0, column=0, columnspan=1, padx=5, pady=10)

    primary_click_opp_button = tk.Button(button_panel, text="Opponent", command=toggle_primary_opponent_action, width=15, height=2)
    primary_click_opp_button.grid(row=0, column=1, columnspan=1, padx=5, pady=10)

currRow += 1

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

            # Resize the image for display
            pil_image = pil_image.resize(
                (pil_image.width * DISPLAY_SCALE_FACTOR, pil_image.height * DISPLAY_SCALE_FACTOR),
                Image.NEAREST
            )

            # Convert the PIL Image to a PhotoImage
            tk_image = ImageTk.PhotoImage(image=pil_image)

            # Update the image_label
            image_label.config(image=tk_image)
            image_label.image = tk_image  # Keep a reference

    # Schedule the next update
    root.after(20, update_image)

def handle_mouse_event(event: tk.Event, data_type: int) -> None:
    global last_click, combined_image

    if combined_image is None:
        return

    # Adjust coordinates based on the display scaling
    x, y = int(event.x / DISPLAY_SCALE_FACTOR), int(event.y / DISPLAY_SCALE_FACTOR)

    if current_mode == "pos_and_rot":
        total_width = combined_image.shape[1]
        half_width = total_width // 2
        if x < 0 or x > total_width or y < 0 or y > combined_image.shape[0]:
            return

        if data_type == POSITION:
            if x >= half_width:
                x_adjusted = x - half_width
                last_click.position = (x_adjusted, y)
                last_click.data_type = POSITION
        elif data_type == OPPONENT_POSITION:
            if x >= half_width:
                x_adjusted = x - half_width
                last_click.position = (x_adjusted, y)
                last_click.data_type = OPPONENT_POSITION
        elif data_type == OPPONENT_ROTATION:
            if x < half_width:
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

def reconnect():
    global client_socket, HOST, PORT
    try:
        client_socket.close()
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        client_socket.connect((HOST, PORT))

        # Send a test message
        client_socket.sendto(b"Init me", (HOST, PORT))

    except Exception as e:
        print("Failed to reconnect:", e)
        # Optionally shutdown or alert user here
        return False
    return True

# Thread to receive data from the server
def receive_thread() -> None:
    global latest_image, combined_image, client_socket
    MAX_RETRY = 5
    retry_count = 0

    while True:
        try:
            data, _ = client_socket.recvfrom(MAX_BUFFER_SIZE)
            retry_count = 0  # Reset retry count on success
        except Exception as e:
            print("Error receiving data:", e)
            if retry_count < MAX_RETRY:
                time.sleep(1)  # Wait before retrying
                retry_count += 1
                continue
            else:
                print("Max retries reached. Attempting to reconnect.")
                reconnect()  # Assume a function to handle reconnection
                retry_count = 0  # Reset retry count
                continue

        if len(data) < 4:
            print("Error: Incomplete size data received")
            root.after(0, send_data_to_robot_controller)
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
            gui_updates,
        ) = parsed_data

        try:
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

        except Exception as e:
            print("Error processing received data:", e)

        # Schedule GUI updates in the main thread
        root.after(0, update_gui_elements, gui_updates)

        # Schedule send_data_to_robot_controller in the main thread
        root.after(0, send_data_to_robot_controller)

# Handle window close event to close the socket
def on_closing() -> None:
    global client_socket
    if client_socket is not None:
        client_socket.close()
    root.destroy()

def update_gui_elements(gui_updates: dict):
    global ignore_callbacks
    if not gui_updates:
        return

    ignore_callbacks = True

    if 'fg_min_delta' in gui_updates:
        fg_min_delta.set(gui_updates['fg_min_delta'])
    if 'foreground_min_delta_slider' in gui_updates:
        foreground_min_delta_slider.set(gui_updates['foreground_min_delta_slider'])
    if 'bg_heal_rate' in gui_updates:
        bg_heal_rate.set(gui_updates['bg_heal_rate'])
    if 'background_heal_rate_slider' in gui_updates:
        background_heal_rate_slider.set(gui_updates['background_heal_rate_slider'])
    if 'force_heal_var' in gui_updates:
        force_heal_var.set(gui_updates['force_heal_var'])
    ignore_callbacks = False
    gui_updates.clear()



# Try to connect to server
connect_to_server()

# Start the image update loop
update_image()

root.protocol("WM_DELETE_WINDOW", on_closing)

# Start the Tkinter main loop
# If it exits for whatever reason, restart it
while True:
    root.mainloop()







