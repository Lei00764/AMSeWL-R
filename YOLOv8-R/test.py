from ultralytics import YOLO

model = YOLO(r".\model.pt")  # YOLOv8-R


# 使用模型
model.val(
    data=r".\ultralytics\cfg\data\data.yaml",
    split="test",
    workers=0,
    save_json=True,
)
