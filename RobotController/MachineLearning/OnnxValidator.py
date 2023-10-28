import onnxruntime as rt
import numpy as np
import cv2
import os

# Load the ONNX model
providers = ['CPUExecutionProvider']
sess = rt.InferenceSession("rotation_model.onnx", providers=providers)
input_name = sess.get_inputs()[0].name


# go through every file in the folder
for filename in os.listdir('./TrainingData/TrainingInputsProjected/'):
    path = './TrainingData/TrainingInputsProjected/' + filename
    # Load and preprocess an image
    img = cv2.imread(path)

    img = img.astype(np.float32) / 255.0
    # img = np.transpose(img, (2, 0, 1))
    # swap bgr to rgb
    cv2.cvtColor(img, cv2.COLOR_BGR2RGB, img)

    img = np.expand_dims(img, axis=0)
    # convert to B x C x H x W
    img = np.transpose(img, (0, 3, 1, 2))

    print("this image: " + path)
    # print the img shape
    print(img)

    # Predict
    result = sess.run(None, {input_name: img})
    print(result)

    result = result[0][0][0]

    # draw and show
    arrow_start = (128 // 2, 128 // 2)
    arrow_end = (
        int(arrow_start[0] + 50 * np.cos(2 * np.pi * result)),
        int(arrow_start[1] + 50 * np.sin(2 * np.pi * result))
    )
    img = cv2.imread(path)

    img = cv2.arrowedLine(img, arrow_start, arrow_end, (0, 255, 0), 2)
    cv2.imshow("Prediction", img)
    cv2.waitKey(0)
