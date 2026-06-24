from ultralytics import YOLO
import time
import numpy as np

# 1. Load the newly created ONNX model
onnx_path = '/home/anujjj_k/drone_ws/src/human_detection/human_detection/best.onnx'
model = YOLO(onnx_path, task='detect')

# 2. Create a dummy image that mimics your Gazebo camera (640x640 pixels)
dummy_frame = np.random.randint(0, 255, (640, 640, 3), dtype=np.uint8)

print("🔥 Warming up the GPU...")
# Run a few dummy predictions to get the hardware spun up
for _ in range(10):
    model.predict(dummy_frame, verbose=False)

print("⏱️ Running FPS Benchmark (100 frames)...")
frames_to_test = 100
start_time = time.time()

# 3. The actual stress test loop
for _ in range(frames_to_test):
    results = model.predict(dummy_frame, verbose=False)

end_time = time.time()

# 4. Calculate results
total_time = end_time - start_time
fps = frames_to_test / total_time

print("====================================")
print(f"Total Time for 100 frames : {total_time:.2f} seconds")
print(f"🚀 Average Inference Speed: {fps:.2f} FPS")
print("====================================")
