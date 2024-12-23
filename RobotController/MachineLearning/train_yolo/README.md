# Yolo Model Training

Open in VSCode, make sure the `MachineLearning` directory has the `TestingData` directory with data inside

Go to the `DataPreprocessor` directory, run `./buildPreprocessor.bat` and `./build/Release/DataPreprocessor.exe` to warp the images to 720x720

Go to the `train_yolo` directory, activate a virtual environment, and run `pip install -r requirements.txt` to install all dependencies.

Edit `data_generation.py` with the train and test size, as well as the directory name for the training data, make sure this matches the name in `data.yaml`.

Run `data_generation.py` to format, augment, and grayscale images and create the file hierarchy

Edit `train_yolo.py` with the device to run training on (cuda or cpu) and run `python train_yolo.py` to train.
