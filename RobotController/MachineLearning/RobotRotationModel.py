import glob
import os
import json
import numpy as np
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Conv2D, MaxPooling2D, Flatten, Dense
from tensorflow.keras.optimizers import Adam
from tensorflow.keras.preprocessing.image import ImageDataGenerator
from tensorflow.keras.callbacks import EarlyStopping
from tensorflow.keras.preprocessing.image import img_to_array, load_img
from keras.layers import Dropout
import tensorflow as tf

# Constants
MODEL_NAME = "rotationDetector.h5"
DATA_PATH = "./TrainingData/"
IMG_SIZE = (128, 128)
BATCH_SIZE = 32
VALIDATION_SPLIT = 0.1

# Get sorted list of image files and corresponding json files
img_files = sorted(glob.glob(os.path.join(DATA_PATH, "TrainingInputsProjected", "image_*.jpg")))
json_files = sorted(glob.glob(os.path.join(DATA_PATH, "TrainingKeys", "image_*.json")))

# Preload JSON Data
labels_data = []
for j in json_files:
    with open(j, 'r') as f:
        data = json.load(f)
        rotation_y = data["rotation"] / 360.0  # normalized
    labels_data.append(rotation_y)
labels_data = np.array(labels_data)

print("Found {} images and {} json files.".format(len(img_files), len(json_files)))

def custom_data_gen(img_files, labels_data, target_size, batch_size, subset):
    # Determine split indices for training and validation
    split_idx = int(len(img_files) * (1.0 - VALIDATION_SPLIT))
    if subset == "training":
        img_files = img_files[:split_idx]
        labels_data = labels_data[:split_idx]
    elif subset == "validation":
        img_files = img_files[split_idx:]
        labels_data = labels_data[split_idx:]
        
    while True:        
        for i in range(0, len(img_files), batch_size):
            batch_img_files = img_files[i:i+batch_size]
            imgs = [img_to_array(load_img(f, target_size=target_size)) / 255.0 for f in batch_img_files]
            labels = labels_data[i:i+batch_size]
            yield np.array(imgs), labels

train_gen = custom_data_gen(img_files, labels_data, IMG_SIZE, BATCH_SIZE, "training")
val_gen = custom_data_gen(img_files, labels_data, IMG_SIZE, BATCH_SIZE, "validation")

# Calculate steps per epoch and validation steps
steps_per_epoch = int(len(glob.glob(os.path.join(DATA_PATH, "TrainingInputs", "image_*.jpg"))) * 0.9) // BATCH_SIZE
val_steps = int(len(glob.glob(os.path.join(DATA_PATH, "TrainingInputs", "image_*.jpg"))) * 0.1) // BATCH_SIZE

# Model Architecture
model = Sequential([
    Conv2D(32, (3, 3), activation='relu', input_shape=(*IMG_SIZE, 3)),
    MaxPooling2D(2, 2),
    Conv2D(64, (3, 3), activation='relu'),
    MaxPooling2D(2, 2),
    Conv2D(128, (3, 3), activation='relu'),
    MaxPooling2D(2, 2),
    Flatten(),
    Dense(512, activation='relu'),
    Dropout(0.5),
    Dense(512, activation='relu'),
    Dropout(0.5),
    Dense(1)  # Single output neuron for normalized rotation y
])

# Compile the model
model.compile(optimizer=Adam(), loss='mse', metrics=['mae'])

# load weights
try:
    model.load_weights(MODEL_NAME)
    print("Loaded weights from file!")
except OSError:
    print("No weights file found!")

def save_h5_model(model):
    model.save(MODEL_NAME)
    print("Saved model to disk!")

# def save_frozen_model(model):
#     import tensorflow as tf
#     from tensorflow.python.framework import convert_to_constants

#     model.save("saved_model_directory")
#     # Load the model from the SavedModel directory
#     loaded = tf.saved_model.load('saved_model_directory')
#     infer = loaded.signatures["serving_default"]

#     # Convert to a frozen graph
#     frozen_func = convert_to_constants.convert_variables_to_constants_v2(infer)
#     frozen_func.graph.as_graph_def()

#     # Save the frozen graph
#     with tf.io.gfile.GFile('frozen_model.pb', 'wb') as f:
#         f.write(frozen_func.graph.as_graph_def().SerializeToString())

def save_onnx_model(model):
    import tf2onnx.convert
    import onnx

    # Specify the input node names that you want to be in NCHW format
    input_names_to_convert = ["conv2d_input"]  # Replace "input_node_name" with the actual names

    onnx_model, _ = tf2onnx.convert.from_keras(model, inputs_as_nchw=input_names_to_convert)
    onnx.save(onnx_model, "model.onnx")

def train_model():
    # Set up early stopping
    early_stop = EarlyStopping(monitor='val_loss', patience=15, restore_best_weights=True)

    try:
        # Train the model
        model.fit(
            train_gen,
            epochs=50,
            steps_per_epoch=steps_per_epoch,
            validation_data=val_gen,
            validation_steps=val_steps,
            callbacks=[early_stop]
        )
    except KeyboardInterrupt:
        print("Training stopped!")
        pass

    save_h5_model(model)

    print("Training completed and model saved!")


##### VISUALIZATION #####
def visualize_predictions(val_gen, model, num_samples=10, grid_dims=(2, 5)):
    import cv2

    """
    Visualize predictions vs. true labels for the validation set.
    
    Args:
    - val_gen: Validation data generator
    - model: Trained model
    - num_samples: Number of samples to visualize
    - grid_dims: Tuple specifying the grid dimensions for display (rows, cols)
    """
    # images, true_labels = next(val_gen) -> gets one
    # get all validation images, true_labels
    images = []
    true_labels = []
    while len(images) < num_samples:
        val_images, labels = next(val_gen)
        images.extend(val_images)
        true_labels.extend(labels)
    
    images = np.array(images)
    true_labels = np.array(true_labels)

    # Create a blank canvas for the grid
    canvas = np.zeros((IMG_SIZE[0] * grid_dims[0], IMG_SIZE[1] * grid_dims[1], 3), dtype=np.uint8)

    # Get predictions
    predicted_labels = model.predict(images)
    
    for idx in range(min(num_samples, len(images))):
        image = (images[idx] * 255).astype(np.uint8)
        # bgr -> rgb
        cv2.cvtColor(image, cv2.COLOR_BGR2RGB, image)
        
        # Arrow start point (center of image)
        arrow_start = (IMG_SIZE[0] // 2, IMG_SIZE[1] // 2)
        
        # Arrow end point for true labels
        arrow_end_true = (
            int(arrow_start[0] + 50 * np.cos(2 * np.pi * true_labels[idx])),
            int(arrow_start[1] + 50 * np.sin(2 * np.pi * true_labels[idx]))
        )
        
        # Arrow end point for predicted labels
        arrow_end_pred = (
            int(arrow_start[0] + 50 * np.cos(2 * np.pi * predicted_labels[idx])),
            int(arrow_start[1] + 50 * np.sin(2 * np.pi * predicted_labels[idx]))
        )
        
        # Draw arrows on the image
        image_with_arrows = cv2.arrowedLine(image, arrow_start, arrow_end_true, (0, 255, 0), 2)  # True in green
        image_with_arrows = cv2.arrowedLine(image_with_arrows, arrow_start, arrow_end_pred, (0, 0, 255), 2)  # Predicted in red
        
        # Determine position on canvas
        row, col = divmod(idx, grid_dims[1])
        canvas[row * IMG_SIZE[0]:(row + 1) * IMG_SIZE[0], col * IMG_SIZE[1]:(col + 1) * IMG_SIZE[1]] = image_with_arrows

    # Show canvas image
    cv2.imshow("Predictions", canvas)
    cv2.waitKey(0)
    cv2.destroyAllWindows()



save_onnx_model(model)

# train_model()

# Call the visualization function after training:
visualize_predictions(val_gen, model, 100, (10, 10))

