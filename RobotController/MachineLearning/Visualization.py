import os
import glob
import numpy as np
from tensorflow.keras.preprocessing.image import load_img, img_to_array
from typing import Tuple
import tensorflow as tf

def network_output_to_angle(output: np.ndarray) -> np.ndarray:
    # if only one element, add a dimension
    if len(output.shape) == 1:
        output = np.expand_dims(output, axis=0) 

    if output.shape[1] != 2:
        raise ValueError("angle_components must have shape (N, 2)")

    # the model outputs two values, one for cos and one for sin
    x_components, y_components = output[:, 0], output[:, 1]
    x_components = (x_components * 2) - 1
    y_components = (y_components * 2) - 1

    # get the angle in radians
    # and multiply by 0.5 since we multiplied all the labels by 2 to map to 0-360
    angles = np.arctan2(y_components, x_components) * 0.5
    return angles

def visualize_rotation_predictions(val_gen, model, num_samples=10, grid_dims=(2, 5),
                                   IMG_SIZE: Tuple[int, int] = (128, 128), start_index=0):
    import cv2
    import numpy as np

    """
    Visualize predictions vs. true labels for the validation set.
    
    Args:
    - val_gen: Validation data generator
    - model: Trained model
    - num_samples: Number of samples to visualize
    - grid_dims: Tuple specifying the grid dimensions for display (rows, cols)
    - IMG_SIZE: The input size to the model
    - start_index: The index to start visualization from in the validation set
    """
    images = []
    true_labels = []
    samples_collected = 0
    samples_skipped = 0

    # Loop through the generator to collect the required samples
    while samples_collected < num_samples:
        try:
            val_images, labels = next(val_gen)
        except StopIteration:
            print("Reached the end of the validation generator.")
            break

        batch_size = len(val_images)

        # Skip batches until we reach the start_index
        if samples_skipped + batch_size <= start_index:
            samples_skipped += batch_size
            continue
        else:
            # Calculate the starting point within the current batch
            start_within_batch = max(0, start_index - samples_skipped)
            val_images = val_images[start_within_batch:]
            labels = labels[start_within_batch:]
            samples_skipped += start_within_batch + len(val_images)

            # Add samples to the list until num_samples is reached
            for img, label in zip(val_images, labels):
                if samples_collected < num_samples:
                    images.append(img)
                    true_labels.append(label)
                    samples_collected += 1
                else:
                    break

    images = np.array(images)
    # Ensure images have the correct shape
    if len(images.shape) == 3:
        images = np.expand_dims(images, axis=-1)

    # Create a blank canvas for the grid
    canvas = np.zeros((IMG_SIZE[0] * grid_dims[0],
                       IMG_SIZE[1] * grid_dims[1], 3), dtype=np.uint8)

    # Get predictions
    predictions = model.predict(images)
    predictions = network_output_to_angle(predictions)

    # Convert true labels to radians
    true_labels = network_output_to_angle(np.array(true_labels))

    for idx in range(min(num_samples, len(images))):
        image = (images[idx] * 255).astype(np.uint8)
        # Convert grayscale to RGB if necessary
        if image.shape[-1] == 1 or len(image.shape) == 2:
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)

        # Arrow start point (center of image)
        arrow_start = (IMG_SIZE[0] // 2, IMG_SIZE[1] // 2)

        # Arrow end point for true labels
        arrow_end_true = (
            int(arrow_start[0] + 50 * np.cos(true_labels[idx])),
            int(arrow_start[1] + 50 * np.sin(true_labels[idx]))
        )

        # Arrow end point for predicted labels
        arrow_end_pred = (
            int(arrow_start[0] + 50 * np.cos(predictions[idx])),
            int(arrow_start[1] + 50 * np.sin(predictions[idx]))
        )

        # Draw arrows on the image
        image_with_arrows = cv2.arrowedLine(
            image, arrow_start, arrow_end_true, (0, 255, 0), 2)  # True in green
        image_with_arrows = cv2.arrowedLine(
            image_with_arrows, arrow_start, arrow_end_pred, (0, 0, 255), 2)  # Predicted in red

        # Determine position on canvas
        row, col = divmod(idx, grid_dims[1])
        canvas[row * IMG_SIZE[0]:(row + 1) * IMG_SIZE[0],
               col * IMG_SIZE[1]:(col + 1) * IMG_SIZE[1]] = image_with_arrows

    # Show canvas image
    cv2.imshow("Predictions", canvas)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def visualize_testing_rotations(model: tf.keras.Model, TESTING_PATH: str, IMG_SIZE: Tuple[int, int]):
    """
    A utility function for visualizing the predictions of a model on the testing data.
    
    Parameters:
    - model: The model to use for predictions.
    - TESTING_PATH: The path to the testing data.
    - IMG_SIZE: The model's input image size.
    """
    # now lets predict on test data
    # Get sorted list of image files and corresponding json files
    img_files = sorted(glob.glob(os.path.join(TESTING_PATH, "image_*.jpg")))

    # there are no labels
    for i in range(len(img_files)):
        import cv2
        img = load_img(
            img_files[i], target_size=IMG_SIZE, color_mode='grayscale')
        img_array = img_to_array(img)
        img_array /= 255.0
        img_array = np.expand_dims(img_array, axis=0)
        predicted_label = network_output_to_angle(model.predict(img_array)[0])
        print("Image: {} Predicted: {}".format(img_files[i], predicted_label))

        # draw arrow
        image = (img_array[0] * 255).astype(np.uint8)
        # bgr -> rgb
        cv2.cvtColor(image, cv2.COLOR_BGR2RGB, image)

        # Arrow start point (center of image)
        arrow_start = (IMG_SIZE[0] // 2, IMG_SIZE[1] // 2)

        # Arrow end point for predicted labels
        arrow_end_pred = (
            int(arrow_start[0] + 50 * np.cos(predicted_label)),
            int(arrow_start[1] + 50 * np.sin(predicted_label))
        )

        # Draw arrows on the image
        image_with_arrows = cv2.arrowedLine(
            image, arrow_start, arrow_end_pred, (0, 0, 255), 2)  # Predicted in red

        # Show canvas image
        cv2.imshow("Predictions", image_with_arrows)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


def visualize_position_predictions(val_gen, model, num_samples=10, grid_dims=(2, 5), IMG_SIZE: Tuple[int, int] = (128, 128)):
    import cv2
    import numpy as np

    """
    Visualize predictions vs. true labels for the validation set of the position model.
    
    Args:
    - val_gen: Validation data generator
    - model: Trained model
    - num_samples: Number of samples to visualize
    - grid_dims: Tuple specifying the grid dimensions for display (rows, cols)
    - IMG_SIZE: The input size to the model
    """

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
    canvas = np.zeros((IMG_SIZE[0] * grid_dims[0],
                      IMG_SIZE[1] * grid_dims[1], 3), dtype=np.uint8)

    # Get predictions
    predicted_positions = model.predict(images)

    for idx in range(min(num_samples, len(images))):
        image = (images[idx] * 255).astype(np.uint8)
        # bgr -> rgb
        cv2.cvtColor(image, cv2.COLOR_BGR2RGB, image)

        # True and Predicted positions
        true_position = (int(true_labels[idx][0] * IMG_SIZE[0]),
                         int(true_labels[idx][1] * IMG_SIZE[1]))
        predicted_position = (int(predicted_positions[idx][0] * IMG_SIZE[0]),
                              int(predicted_positions[idx][1] * IMG_SIZE[1]))

        # Draw circles on the image
        cv2.circle(image, true_position, 5, (0, 255, 0), 2)        # True in green
        cv2.circle(image, predicted_position, 5, (0, 0, 255), 2)  # Predicted in red

        # Determine position on canvas
        row, col = divmod(idx, grid_dims[1])
        canvas[row * IMG_SIZE[0]:(row + 1) * IMG_SIZE[0], col *
               IMG_SIZE[1]:(col + 1) * IMG_SIZE[1]] = image

    # Show canvas image
    cv2.imshow("Predictions", canvas)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
