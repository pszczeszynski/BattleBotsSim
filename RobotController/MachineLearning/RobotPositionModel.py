import glob
import os
import json
import numpy as np
from keras.models import Sequential
from keras.layers import Conv2D, MaxPooling2D, Flatten, Dense
from keras.optimizers import Adam
from keras.callbacks import EarlyStopping
from keras.layers import Dropout
from Visualization import visualize_position_predictions
from Utilities import Augmentations, save_onnx_model, save_h5_model, custom_data_gen

# Constants
MODEL_NAME = "positionDetector.h5"
DATA_PATH = "./TrainingData/"
TESTING_PATH = "./TestingData/TestingInputs/"
IMG_SIZE = (360, 360)#(1280, 720)
BATCH_SIZE = 128
VALIDATION_SPLIT = 0.1
EARLY_STOPPING_PATIENCE = 15

# Get sorted list of image files and corresponding json files
img_files = sorted(glob.glob(os.path.join(
    DATA_PATH, "TrainingInputsPosition", "image_*.jpg")))
json_files = sorted(glob.glob(os.path.join(
    DATA_PATH, "TrainingKeys", "image_*.json")))

# Preload JSON Data
labels_data = []
for j in json_files:
    with open(j, 'r') as f:
        data = json.load(f)
        this_position = np.array([data["transformedPoint"]["x"] / 720, data["transformedPoint"]["y"] / 720])

    labels_data.append(this_position)
labels_data = np.array(labels_data)

print("Found {} images and {} json files.".format(
    len(img_files), len(json_files)))

# add augmentations to make the model more robust
augmentations = Augmentations(
    zoom_range=0.0,
    width_shift_range=0.0,
    height_shift_range=0.0,
    rotation_range=0.0
)

train_gen = custom_data_gen(img_files, labels_data,
                            IMG_SIZE, BATCH_SIZE, "training",
                            VALIDATION_SPLIT, augmentations)
val_gen = custom_data_gen(img_files, labels_data,
                          IMG_SIZE, BATCH_SIZE, "validation", VALIDATION_SPLIT)

# Calculate steps per epoch and validation steps
steps_per_epoch = int(len(glob.glob(os.path.join(
    DATA_PATH, "TrainingInputsPosition", "image_*.jpg"))) * 0.9) // BATCH_SIZE
val_steps = int(len(glob.glob(os.path.join(
    DATA_PATH, "TrainingInputsPosition", "image_*.jpg"))) * 0.1) // BATCH_SIZE

# Model Architecture
model = Sequential([
    Conv2D(16, (3, 3), activation='relu', input_shape=(*IMG_SIZE, 1)),
    MaxPooling2D(2, 2),
    Conv2D(32, (3, 3), activation='relu'),
    MaxPooling2D(2, 2),
    Conv2D(64, (3, 3), activation='relu'),
    MaxPooling2D(2, 2),
    Flatten(),
    Dense(256, activation='relu'),
    Dropout(0.5),
    Dense(128, activation='relu'),
    Dropout(0.5),
    Dense(2)
])

# Now compile the model with this custom loss
model.compile(optimizer=Adam(), loss="mse", metrics=['mae'])

# load weights
try:
    model.load_weights(MODEL_NAME)
    print("Loaded weights from file!")
except OSError:
    print("No weights file found!")


def train_model():
    # Set up early stopping
    early_stop = EarlyStopping(
        monitor='val_loss', patience=EARLY_STOPPING_PATIENCE, restore_best_weights=True)

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

    save_h5_model(model, MODEL_NAME)

    print("Training completed and model saved!")


##### VISUALIZATION #####
train_model()
save_onnx_model(model, "position_model.onnx")

# # Call the visualization function after training:
# visualize_predictions(val_gen, model, 100, (10, 10), IMG_SIZE)

visualize_position_predictions(val_gen, model, 10, (5, 2), IMG_SIZE)
