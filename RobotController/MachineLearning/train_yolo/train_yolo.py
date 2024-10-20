import ultralytics

MODEL_START = "yolov8n.yaml"
DATASET_PATH = "data.yaml"

model = ultralytics.YOLO(MODEL_START)

if __name__ == '__main__':
    model.train(
        data=DATASET_PATH,
        epochs=50,
        patience=10,
        imgsz=720,     # images are 720 x 720
        device=0,   # mps for mac gpu, device=0 for cuda, device=0,1,2,3 for multiple gpu cores,  or device="cpu"
        workers=8,
        verbose=True,
    )

