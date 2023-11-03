import ultralytics

MODEL_START = "yolov8n.yaml"
DATASET_PATH = "data.yaml"

model = ultralytics.YOLO(MODEL_START)

model.train(
    data=DATASET_PATH,
    imgsz=1280,
    device="mps",
    workers=8,
    verbose=True,
)

print("Done!")
