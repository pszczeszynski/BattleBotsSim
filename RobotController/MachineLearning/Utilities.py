from typing import List, Tuple, Union
import numpy as np
from tensorflow.keras.preprocessing.image import ImageDataGenerator, img_to_array, load_img
import tensorflow as tf
import cv2

def save_onnx_model(model: tf.keras.Model, name: str):
    """
    A utility function for saving a Keras model as an ONNX model.

    Parameters:
    - model: The Keras model to save.
    - name: The name of the ONNX model file.
    """
    import tf2onnx.convert
    import onnx

    # Specify the input node names that you want to be in NCHW format
    # Replace "input_node_name" with the actual names
    input_names_to_convert = ["conv2d_input"]

    onnx_model, _ = tf2onnx.convert.from_keras(
        model, inputs_as_nchw=input_names_to_convert)
    onnx.save(onnx_model, name)


def save_h5_model(model: tf.keras.Model, name: str):
    model.save(name)
    print("Saved model to disk!")


class Augmentations:
    def __init__(self, zoom_range: float = 0.0, width_shift_range: float = 0.0,
                 height_shift_range: float = 0.0, rotation_range: float = 0,
                 brightness_range: List[float] = [1.0, 1.0]):
        self.zoom_range = zoom_range
        self.width_shift_range = width_shift_range
        self.height_shift_range = height_shift_range
        self.rotation_range = rotation_range
        self.brightness_range = brightness_range


def custom_data_gen(img_files: List[str],
                    labels_data: List[Union[int, float]],
                    target_size: Tuple[int, int],
                    batch_size: int, subset: str,
                    validation_split: float,
                    augmentations: Augmentations = Augmentations()) -> Tuple[np.ndarray, List[Union[int, float]]]:
    """
    A custom data generator for batching and yielding image data and
    corresponding labels.
    
    Parameters:
    - img_files: List of file paths to images.
    - labels_data: List of labels corresponding to each image file.
    - target_size: Tuple representing the desired image dimensions (height, width).
    - batch_size: Number of images to yield per batch.
    - subset: Specifies whether the data should be split for "training" or "validation".
    - validation_split: The percentage of data to use for validation.
    
    Returns:
    - A tuple containing a batch of image data as a numpy array and a list of corresponding labels.
    """
    # Data Augmentation for training data
    if subset == "training":
        augmentation = ImageDataGenerator(
            zoom_range=augmentations.zoom_range,
            width_shift_range=augmentations.width_shift_range,
            height_shift_range=augmentations.height_shift_range,
            fill_mode='constant',
            rotation_range=augmentations.rotation_range,
            brightness_range=[0.75, 1.25]
        )
    else:
        # No augmentation for validation data, just rescaling
        augmentation = ImageDataGenerator()

    # Determine split indices for training and validation
    split_idx = int(len(img_files) * (1.0 - validation_split))
    if subset == "training":
        img_files = img_files[:split_idx]
        labels_data = labels_data[:split_idx]
    elif subset == "validation":
        img_files = img_files[split_idx:]
        labels_data = labels_data[split_idx:]

    while True:
        for i in range(0, len(img_files), batch_size):
            batch_img_files = img_files[i:i+batch_size]

            # for each image in the new batch
            imgs = []
            for f in batch_img_files:
                # load it, convert to grayscale
                img = load_img(f, target_size=target_size,
                               color_mode='grayscale')
                # convert to numpy array, normalize and append
                img_array = img_to_array(img)
                # apply augmentations
                augmented_img = augmentation.random_transform(img_array)
                augmented_img /= 255.0

                # clip from 0 to 1
                augmented_img = np.clip(augmented_img, 0, 1)

                while np.random.rand() < 0.5:
                    augmented_img = cv2.blur(augmented_img, (5, 5))
                    # make sure the image is still the same size
                    augmented_img = augmented_img[:target_size[0], :target_size[1]]
                    # Ensure the image has the expected number of channels (e.g., 1 for grayscale)
                    augmented_img = np.expand_dims(augmented_img, axis=-1)

                imgs.append(augmented_img)

            labels = labels_data[i:i+batch_size]
            yield np.array(imgs), labels
