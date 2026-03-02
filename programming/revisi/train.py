from ultralytics import YOLO

# TODO: Change this
DATASET_DIR = './annotated-dataset/My First Project.v2i.yolov11'

model = YOLO("yolo11n.pt")
model.train(
  data=DATASET_DIR + '/data.yaml',
  imgsz=512,
  epochs=100,
  batch=-1,
  device="cpu",
  project=DATASET_DIR,
  name="train",
  workers=1,
  seed=69,
)

best_model = model.trainer.best
export_model = YOLO(best_model)
export_model.export(format="onnx", opset=12, imgsz=512)