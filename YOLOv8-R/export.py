import warnings

warnings.filterwarnings("ignore")
from ultralytics import YOLO

# onnx onnxsim onnxruntime onnxruntime-gpu

if __name__ == "__main__":
    model = YOLO("model.pt")
    success = model.export(format="onnx", batch=1)
