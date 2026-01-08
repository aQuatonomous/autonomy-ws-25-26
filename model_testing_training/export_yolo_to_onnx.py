# export_yolo_to_onnx.py
from ultralytics import YOLO

# Load your trained model
model = YOLO('/home/lorenzo/computer_vision/weights.pt')

# Export to ONNX with settings optimized for TensorRT
model.export(
    format='onnx',           # Export format
    imgsz=640,               # Input size (matches your training)
    opset=13,                # ONNX opset version (safe for TensorRT 10.x)
    simplify=True,           # Simplify ONNX model for better TensorRT compatibility
    dynamic=False,           # Fixed batch size (faster on Jetson)
    half=False,              # Keep FP32 for ONNX (we'll use FP16 in TensorRT)
)

print("\n Successfully exported to ONNX!")
print("  Output file: aqua_weights.onnx")