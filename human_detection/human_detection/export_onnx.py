from ultralytics import YOLO

# 1. Load your custom trained PyTorch model
model_path = '/home/anujjj_k/drone_ws/src/human_detection/human_detection/best.pt'
model = YOLO(model_path)

print("🚀 Exporting model to ONNX format...")

# 2. Export to ONNX
# This strips away the training data and optimizes the graph for pure speed
exported_path = model.export(format='onnx')

print(f"✅ Export complete! Your high-speed model is saved at: {exported_path}")
