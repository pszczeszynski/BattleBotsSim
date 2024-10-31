import glob
import os
import json
import numpy as np
from keras.models import Sequential
from keras.layers import Conv2D, MaxPooling2D, Flatten, Dense
from keras.optimizers import Adam
from keras.callbacks import EarlyStopping
from keras.layers import Dropout
from Visualization import visualize_rotation_predictions, visualize_testing_rotations
from Utilities import Augmentations, save_onnx_model, save_h5_model, custom_data_gen
from keras.callbacks import ReduceLROnPlateau

# Constants
MODEL_NAME = "rotationDetector_hoop.h5"
DATA_PATH = "./TrainingData/"
TESTING_PATH = "./TestingData/TestingInputs/"
IMG_SIZE = (128, 128)
BATCH_SIZE = 16
VALIDATION_SPLIT = 0.1
EARLY_STOPPING_PATIENCE = 7
REDUCE_LR_PATIENCE = 3
REDUCE_LR_FACTOR = 0.3

# Get sorted list of image files and corresponding json files
img_files = sorted(glob.glob(os.path.join(
    DATA_PATH, "TrainingInputsProjected", "image_*.jpg")))
json_files = sorted(glob.glob(os.path.join(
    DATA_PATH, "TrainingKeys", "image_*.json")))

# Preload JSON Data
labels_data = []
for j in json_files:
    with open(j, 'r') as f:
        data = json.load(f)

    # Take the rotation mod 180 since the robot is symmetrical, then multiply by
    # 2 so that the range is 0-360. Multiplying by 2 is necessary so that the
    # sin and cos components don't have any discontinuities.
    label = (data["rotation"] % 180) * 2 * (np.pi / 180.0)
    # get cos and sin components
    label = np.array([(np.cos(label) + 1) / 2.0, (np.sin(label) + 1) / 2.0])
    # add to labels_data
    labels_data.append(label)


# convert to numpy array
labels_data = np.array(labels_data)
print("labels_data shape: ", labels_data.shape)

print("Found {} images and {} json files.".format(
    len(img_files), len(json_files)))

# add augmentations to make the model more robust
augmentations = Augmentations(
    zoom_range=0.3,
    width_shift_range=0.1,
    height_shift_range=0.1,
    rotation_range=0.0,
    brightness_range=(0.75, 1.25),
    max_overlay_objects=10,
    object_size=(10, 10),
    blur_probability=0.3
)

train_gen = custom_data_gen(img_files, labels_data,
                            IMG_SIZE, BATCH_SIZE, "training",
                            VALIDATION_SPLIT, augmentations)
val_gen = custom_data_gen(img_files, labels_data,
                          IMG_SIZE, BATCH_SIZE, "validation", VALIDATION_SPLIT, augmentations)

# Calculate steps per epoch and validation steps
steps_per_epoch = int(len(glob.glob(os.path.join(
    DATA_PATH, "TrainingInputsProjected", "image_*.jpg"))) * 0.9) // BATCH_SIZE
val_steps = int(len(glob.glob(os.path.join(
    DATA_PATH, "TrainingInputsProjected", "image_*.jpg"))) * 0.1) // BATCH_SIZE

# Model Architecture
model = Sequential([
    Conv2D(64, (3, 3), activation='relu', input_shape=(*IMG_SIZE, 1)),
    MaxPooling2D(2, 2),
    Conv2D(128, (3, 3), activation='relu'),
    MaxPooling2D(2, 2),
    Conv2D(256, (3, 3), activation='relu'),
    MaxPooling2D(2, 2),
    Flatten(),
    Dense(1024, activation='relu'),
    Dropout(0.5),
    Dense(1024, activation='relu'),
    Dropout(0.5),
    Dense(2) # 2 output neurons for cos and sin components
])

# Now compile the model with this custom loss
model.compile(optimizer=Adam(learning_rate=0.001), loss="mse", metrics=['mae'])

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

    # Set up learning rate reduction on plateau
    reduce_lr = ReduceLROnPlateau(
        monitor='val_loss', factor=REDUCE_LR_FACTOR,
        patience=REDUCE_LR_PATIENCE,
        verbose=1, mode='min',
        min_delta=0.0001, cooldown=0, min_lr=0)

    try:
        # Train the model
        model.fit(
            train_gen,
            epochs=15,
            steps_per_epoch=steps_per_epoch,
            validation_data=val_gen,
            validation_steps=val_steps,
            callbacks=[early_stop, reduce_lr]  # Add reduce_lr to the callbacks list
        )
    except KeyboardInterrupt:
        print("Training stopped!")
        pass

    save_h5_model(model, MODEL_NAME)

    print("Training completed and model saved!")

##### VISUALIZATION #####
train_model()
save_onnx_model(model, "rotation_model_hoop.onnx")

# Call the visualization function after training:
visualize_rotation_predictions(train_gen, model, 100, (10, 10), IMG_SIZE)

# # visualize some testing rotations
# visualize_testing_rotations(model, TESTING_PATH, IMG_SIZE)
