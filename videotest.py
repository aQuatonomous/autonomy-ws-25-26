from ultralytics import YOLO
import cv2
import numpy as np
import time

model = YOLO('/home/lorenzo/computer_vision/weights.pt')

# Process video and collect metrics
cap = cv2.VideoCapture("video.mkv")
fps = cap.get(cv2.CAP_PROP_FPS)
total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

# Metrics collection
detection_data = {
    'frame_times': [],
    'detections_per_frame': [],
    'confidence_scores': [],
    'class_distribution': {},
    'detection_stability': []
}

frame_count = 0
prev_detections = 0

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    frame_count += 1
    
    # Time the inference
    start_time = time.time()
    results = model.predict(
        source=frame,
        device=0,
        imgsz=640,
        conf=0.6,
        verbose=False
    )
    inference_time = (time.time() - start_time) * 1000
    
    # Collect metrics
    detection_data['frame_times'].append(inference_time)
    detection_data['detections_per_frame'].append(len(results[0].boxes))
    
    # Track detection stability
    if frame_count > 1:
        stability = abs(len(results[0].boxes) - prev_detections)
        detection_data['detection_stability'].append(stability)
    
    prev_detections = len(results[0].boxes)
    
    # Collect per-detection data
    for box in results[0].boxes:
        conf = float(box.conf[0])
        cls = int(box.cls[0])
        label = results[0].names[cls]
        
        detection_data['confidence_scores'].append(conf)
        
        if label not in detection_data['class_distribution']:
            detection_data['class_distribution'][label] = 0
        detection_data['class_distribution'][label] += 1
    
    # Print progress every 100 frames
    if frame_count % 100 == 0:
        print(f"Processed {frame_count}/{total_frames} frames")

cap.release()

# Calculate performance metrics
print("\n" + "="*60)
print("VIDEO PROCESSING PERFORMANCE ANALYSIS")
print("="*60)

# Timing metrics
avg_inference = np.mean(detection_data['frame_times'])
min_inference = np.min(detection_data['frame_times'])
max_inference = np.max(detection_data['frame_times'])
std_inference = np.std(detection_data['frame_times'])

print(f"\n📊 TIMING METRICS:")
print(f"Average inference time: {avg_inference:.1f}ms")
print(f"Min inference time: {min_inference:.1f}ms")
print(f"Max inference time: {max_inference:.1f}ms")
print(f"Standard deviation: {std_inference:.1f}ms")
print(f"Max theoretical FPS: {1000/min_inference:.1f}")
print(f"Average FPS: {1000/avg_inference:.1f}")

# Detection metrics
avg_detections = np.mean(detection_data['detections_per_frame'])
total_detections = sum(detection_data['detections_per_frame'])
frames_with_detections = sum(1 for x in detection_data['detections_per_frame'] if x > 0)

print(f"\n🎯 DETECTION METRICS:")
print(f"Total frames processed: {frame_count}")
print(f"Frames with detections: {frames_with_detections}")
print(f"Detection rate: {frames_with_detections/frame_count*100:.1f}%")
print(f"Average detections per frame: {avg_detections:.1f}")
print(f"Total detections: {total_detections}")

# Confidence analysis
if detection_data['confidence_scores']:
    avg_confidence = np.mean(detection_data['confidence_scores'])
    min_confidence = np.min(detection_data['confidence_scores'])
    max_confidence = np.max(detection_data['confidence_scores'])
    
    print(f"\n🎯 CONFIDENCE ANALYSIS:")
    print(f"Average confidence: {avg_confidence:.3f}")
    print(f"Min confidence: {min_confidence:.3f}")
    print(f"Max confidence: {max_confidence:.3f}")
    print(f"High confidence detections (>0.8): {sum(1 for x in detection_data['confidence_scores'] if x > 0.8)}")

# Class distribution
print(f"\n📈 CLASS DISTRIBUTION:")
for class_name, count in detection_data['class_distribution'].items():
    percentage = count / total_detections * 100
    print(f"{class_name}: {count} detections ({percentage:.1f}%)")

# Stability analysis
if detection_data['detection_stability']:
    avg_stability = np.mean(detection_data['detection_stability'])
    print(f"\n📊 DETECTION STABILITY:")
    print(f"Average detection count change: {avg_stability:.1f}")
    print(f"Stable detection (change ≤1): {sum(1 for x in detection_data['detection_stability'] if x <= 1)} frames")

print("\n" + "="*60)