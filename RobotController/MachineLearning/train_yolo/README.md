# Yolo Model Training

Open in VSCode, make sure the `MachineLearning` directory has the `TestingData` directory with data inside

Go to the `train_yolo` directory, activate a virtual environment, and run `pip install -r requirements.txt` to install all dependencies.

Open `train_yolo.ipynb`, edit the second cell with the desired train test split and run it to create the yolo training directory structure and annotation files

The third, fourth, and fifth cells are for visualisation, and the first cell is just to check the gpu.

Edit `train_yolo.py` with the device to run training on (cuda or cpu) and run `python train_yolo.py` to train.
