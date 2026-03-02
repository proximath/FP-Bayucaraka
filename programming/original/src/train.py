from ultralytics import YOLO
from pathlib import Path

DATASET_DIR = Path('datasets/My First Project.v2i.yolov11')
DATA_YAML = DATASET_DIR / 'data.yaml'
PRETRAINED_MODEL = "yolo11n.pt"
IMG_SIZE = 512
EPOCHS = 100 # Banyak iterasi
BATCH = -1  # -1 = Ultralytics auto-batch
DEVICE = "cpu" # Use GPU
WORKERS = 1 # Banyak thread yang dipakai
SEED = 42
PROJECT_DIR = DATASET_DIR / "runs"
RUN_NAME = "train"
EXPORT_ONNX = True
ONNX_OPSET = 12
print(DATA_YAML.resolve())

model = YOLO(PRETRAINED_MODEL)
model.train(
    data=str(DATA_YAML.resolve()),
    imgsz=IMG_SIZE,
    epochs=EPOCHS,
    batch=BATCH,
    device=DEVICE,
    project=str(PROJECT_DIR),
    name=RUN_NAME,
    workers=WORKERS,
    seed=SEED,
  )
metrics = model.val()
best_model = model.trainer.best
export_model = YOLO(best_model)
export_model.export(format="onnx", opset=ONNX_OPSET, imgsz=IMG_SIZE)
print(best_model)