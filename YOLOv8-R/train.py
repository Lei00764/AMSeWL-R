import warnings

warnings.filterwarnings("ignore")
from ultralytics import YOLO

if __name__ == "__main__":
    model = YOLO(r".\ultralytics\cfg\models\yolov8-R.yaml")
    model.load("yolov8n.pt")  # loading pretrain weights
    model.train(
        data=r".\ultralytics\cfg\data\data.yaml",
        cache=False,
        imgsz=640,
        epochs=100,
        batch=16,
        close_mosaic=10,
        workers=0,
        device="0",
        optimizer="SGD",  # using SGD
        # resume='', # last.pt path
        amp=True,  # close amp
        # fraction=0.2,
        project="runs/train",
        name="exp",
    )
