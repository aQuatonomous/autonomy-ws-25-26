
# initial train
yolo task=detect \
 mode=train \
 data=data.yaml \
 model=yolo11n.pt \
 imgsz=640 \
 epochs=150 \
 patience=50 \
 batch=16 \
 project=buoy_detection \
 device=0 \
 plots=True \
 workers=2 \
 name=yolov11n_detection1 \
