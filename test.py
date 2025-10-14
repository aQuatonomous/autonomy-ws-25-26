from ultralytics import YOLO

model = YOLO('/home/lorenzo/computer_vision/weights.pt')
res=model.predict("image.png", device='0', imgsz=640, verbose=False) # GPU warmup
res = model.predict(
    source="image.png",
    device='0',
    imgsz=640,
    conf=0.60,
    iou=0.50,
    max_det=30,
    save=True, show=True, save_txt=False, save_conf=False, verbose=False
)
for box in res[0].boxes:
    print(f"Preprocess: {res[0].speed['preprocess']:.2f} ms | "
      f"Inference: {res[0].speed['inference']:.2f} ms | "
      f"Postprocess: {res[0].speed['postprocess']:.2f} ms | "
      f"Total: {sum(res[0].speed.values()):.2f} ms")
    # Convert tensors to Python floats
    x1, y1, x2, y2 = box.xyxy[0].tolist()     # top-left & bottom-right corners
    conf = float(box.conf[0])                 # confidence score (0–1)
    cls = int(box.cls[0])                     # class ID number
    label = res[0].names[cls]                 # human-readable class name

    # Derived values
    x_center = (x1 + x2) / 2
    y_center = (y1 + y2) / 2
    width = x2 - x1
    height = y2 - y1

    print(f"Detected: {label}")
    print(f" Confidence: {conf:.2f}")
    print(f" Bounding box (x1,y1,x2,y2): {x1:.1f}, {y1:.1f}, {x2:.1f}, {y2:.1f}")
    print(f" Center: ({x_center:.1f}, {y_center:.1f}), Size: ({width:.1f}, {height:.1f})")
    print("-" * 40)