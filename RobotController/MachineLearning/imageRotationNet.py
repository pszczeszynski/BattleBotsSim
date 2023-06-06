import tensorflow as tf
from tensorflow import keras
from keras import layers
import matplotlib.pyplot as plt

import numpy as np
from PIL import Image
from noise import pnoise2

SAMPLE_COUNT = 1000
BATCH_SIZE = 100

class PerlinNoise:
    # Image dimensions
    width = 512
    height = 512

    # Perlin noise parameters
    scale = 0.1  # Controls the frequency of the noise
    octaves = 6  # Number of noise layers
    persistence = 0.5  # Controls the amplitude of each octave

    noise_array = None
    def __init__(self, width, height, scale, octaves, persistence):
        self.width = width
        self.height = height
        self.scale = scale
        self.octaves = octaves
        self.persistence = persistence

    def generate(self):
        shift_amount_x = np.random.uniform(0, 1000)
        shift_amount_y = np.random.uniform(0, 1000)

        # Generate Perlin noise
        noise_array = np.zeros((self.height, self.width))
        for y in range(self.height):
            for x in range(self.width):
                fake_x = x + shift_amount_x
                fake_y = y + shift_amount_y
                noise_value = pnoise2(fake_x * self.scale, fake_y * self.scale, octaves=self.octaves, persistence=self.persistence)
                noise_array[y][x] = noise_value

        return noise_array

MAX_ROTATION = 90

def generate_data(sample_count=SAMPLE_COUNT):
    """
    Generates the training data for the image rotation net
    """
    # each input is two images of size 100x100
    # the first image contains a background and some forground object
    # the second image contains the same background and but the forground object is rotated by some angle
    # the key is the angle of rotation

    noise_generator: PerlinNoise = PerlinNoise(100, 100, 0.1, 6, 0.5)

    # circle mask
    # create a circle mask
    circle_mask = np.zeros((100, 100))
    for y in range(100):
        for x in range(100):
            if (x - 50) ** 2 + (y - 50) ** 2 < 50 ** 2:
                circle_mask[y][x] = 1

    for i in range(sample_count):
        # generate perlin noise
        foreground_noise = noise_generator.generate()
        background = noise_generator.generate()
        # normalize
        foreground_noise = (foreground_noise - np.min(foreground_noise)) / (np.max(foreground_noise) - np.min(foreground_noise))
        background = (background - np.min(background)) / (np.max(background) - np.min(background))
        background = background * 255 * 0.2

        # set to 0 outside of the circle
        foreground_noise = foreground_noise * circle_mask
        # clip foreground_noise to 0 when it's below a threshold
        foreground_noise[foreground_noise < 0.5] = 0
        # rotate the perlin noise randomly
        foreground_noise = Image.fromarray(((foreground_noise - 0.5) * 2 * 255).astype(np.uint8))
    
        rotation_amount_deg = np.random.uniform(0, MAX_ROTATION)
        foreground_noise_rotated = foreground_noise.rotate(rotation_amount_deg)

        background = Image.fromarray(background.astype(np.uint8))

        # iterate over every pixel
        for y in range(100):
            for x in range(100):
                if foreground_noise.getpixel((x, y)) < 2:
                    # set background to 0
                    foreground_noise.putpixel((x, y), background.getpixel((x, y)))

                if foreground_noise_rotated.getpixel((x, y)) < 2:
                    # set background to 0
                    foreground_noise_rotated.putpixel((x, y), background.getpixel((x, y)))

        # concat the two images together and save as png
        Image.fromarray(np.concatenate((foreground_noise, foreground_noise_rotated), axis=1)).save("TrainingData/TrainingInputs/sample_" + str(i) + ".png")

        # save the rotation amount in the training keys as it's own file
        np.save("TrainingData/TrainingKeys/sample_" + str(i) + ".npy", rotation_amount_deg)

generate_data(SAMPLE_COUNT)

# 2. construct a keras model that takes in the concatenated images and outputs the rotation amount

# Define the input shape of the concatenated images
input_shape = (100, 100, 2)

# Left half input
left_input = keras.Input(shape=(100, 100, 1))

# Process the left half
left_conv1 = layers.Conv2D(32, kernel_size=(3, 3), activation="relu")(left_input)
left_pool1 = layers.MaxPooling2D(pool_size=(2, 2))(left_conv1)
left_drop1 = layers.Dropout(0.25)(left_pool1)
left_flatten = layers.Flatten()(left_drop1)

# Right half input
right_input = keras.Input(shape=(100, 100, 1))

# Process the right half
right_conv1 = layers.Conv2D(32, kernel_size=(3, 3), activation="relu")(right_input)
right_pool1 = layers.MaxPooling2D(pool_size=(2, 2))(right_conv1)
right_drop1 = layers.Dropout(0.25)(right_pool1)
right_flatten = layers.Flatten()(right_drop1)

# Concatenate the flattened features from left and right halves
concatenated_features = layers.Concatenate()([left_flatten, right_flatten])

# Dense layers
dense1 = layers.Dense(128, activation="relu")(concatenated_features)
drop2 = layers.Dropout(0.5)(dense1)

# Output layer for regression problem
output = layers.Dense(1, activation="linear")(drop2)

# Create the model
model = keras.Model(inputs=[left_input, right_input], outputs=output)

# 3. train the model on the data

# load the data
training_keys = []

# read keys
for i in range(SAMPLE_COUNT):
    training_keys.append(np.load("TrainingData/TrainingKeys/sample_" + str(i) + ".npy") / MAX_ROTATION)

# get image dataset with no labels
training_inputs: tf.data.Dataset = keras.preprocessing.image_dataset_from_directory("TrainingData/TrainingInputs", label_mode=None, \
    image_size=(100, 200), color_mode="grayscale", batch_size=BATCH_SIZE, shuffle=False)

# normalize the images
normalized_images = training_inputs.map(lambda x: tf.divide(x, 255))

# split the images into two halves
left_images = normalized_images.map(lambda x: x[:, :, :100])
right_images = normalized_images.map(lambda x: x[:, :, 100:])

# reshape the training_inputs into batches
training_keys = np.array(training_keys)
training_keys_batched = np.reshape(training_keys, (int(SAMPLE_COUNT / BATCH_SIZE), BATCH_SIZE))

# convert the output keys into a dataset
batched_keys = tf.constant(training_keys_batched)
labels_ds = tf.data.Dataset.from_tensor_slices(batched_keys)

all_data = tf.data.Dataset.zip(((left_images, right_images), labels_ds))

# partition the dataset into training and validation
all_data = all_data.shuffle(SAMPLE_COUNT)
training_data = all_data.take(int(SAMPLE_COUNT / BATCH_SIZE * 0.8))
validation_data = all_data.skip(int(SAMPLE_COUNT / BATCH_SIZE * 0.8))

# calculate the number of batches per epoch for just the validation portion
training_steps_per_epoch = int(SAMPLE_COUNT * 0.8 / BATCH_SIZE)
validation_steps_per_batch = int(SAMPLE_COUNT * 0.2 / BATCH_SIZE)
print("validation_steps_per_batch: " + str(validation_steps_per_batch))

learning_rate = 0.001  # Set your desired learning rate
optimizer = keras.optimizers.Adam(learning_rate=learning_rate)

# compile the model
model.compile(optimizer=optimizer,
                loss=tf.keras.losses.MeanSquaredError(),
                metrics=[tf.keras.metrics.MeanAbsoluteError()])


# train the model
model.fit(training_data, validation_data=validation_data, epochs=1000, verbose=1, \
          steps_per_epoch=training_steps_per_epoch, validation_steps=validation_steps_per_batch)