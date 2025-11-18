# test_inference.py (FIXED VERSION WITH CUDA MEMORY MANAGEMENT)
import tensorrt as trt
import numpy as np
import cv2
from typing import Tuple, List
import time
import ctypes

# CUDA runtime functions via ctypes
class CudaRuntime:
    def __init__(self):
        try:
            self.cuda = ctypes.CDLL("libcudart.so")
        except:
            # Try alternative path
            self.cuda = ctypes.CDLL("libcudart.so.12")
        
        # Define function signatures
        self.cuda.cudaMalloc.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_size_t]
        self.cuda.cudaMalloc.restype = ctypes.c_int
        
        self.cuda.cudaMemcpy.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t, ctypes.c_int]
        self.cuda.cudaMemcpy.restype = ctypes.c_int
        
        self.cuda.cudaFree.argtypes = [ctypes.c_void_p]
        self.cuda.cudaFree.restype = ctypes.c_int
        
        self.cuda.cudaDeviceSynchronize.restype = ctypes.c_int
        
        # Memory copy kinds
        self.cudaMemcpyHostToDevice = 1
        self.cudaMemcpyDeviceToHost = 2
    
    def malloc(self, size):
        """Allocate GPU memory"""
        ptr = ctypes.c_void_p()
        result = self.cuda.cudaMalloc(ctypes.byref(ptr), size)
        if result != 0:
            raise RuntimeError(f"CUDA malloc failed: {result}")
        return ptr.value
    
    def memcpy_htod(self, dst, src, size):
        """Copy from host to device"""
        result = self.cuda.cudaMemcpy(dst, src, size, self.cudaMemcpyHostToDevice)
        if result != 0:
            raise RuntimeError(f"CUDA memcpy H2D failed: {result}")
    
    def memcpy_dtoh(self, dst, src, size):
        """Copy from device to host"""
        result = self.cuda.cudaMemcpy(dst, src, size, self.cudaMemcpyDeviceToHost)
        if result != 0:
            raise RuntimeError(f"CUDA memcpy D2H failed: {result}")
    
    def free(self, ptr):
        """Free GPU memory"""
        if ptr:
            self.cuda.cudaFree(ptr)
    
    def synchronize(self):
        """Synchronize CUDA operations"""
        self.cuda.cudaDeviceSynchronize()


class TensorRTInference:
    def __init__(self, engine_path: str):
        """Initialize TensorRT engine and context"""
        self.logger = trt.Logger(trt.Logger.WARNING)
        self.cuda = CudaRuntime()
        
        # Load engine
        with open(engine_path, "rb") as f:
            engine_data = f.read()
        
        runtime = trt.Runtime(self.logger)
        self.engine = runtime.deserialize_cuda_engine(engine_data)
        self.context = self.engine.create_execution_context()
        
        # Get input/output names and shapes
        self.input_name = None
        self.output_name = None
        
        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            mode = self.engine.get_tensor_mode(name)
            if mode == trt.TensorIOMode.INPUT:
                self.input_name = name
                self.input_shape = self.engine.get_tensor_shape(name)
            else:
                self.output_name = name
                self.output_shape = self.engine.get_tensor_shape(name)
        
        print(f"Input: {self.input_name}, Shape: {self.input_shape}")
        print(f"Output: {self.output_name}, Shape: {self.output_shape}")
        
        # Allocate GPU memory
        self.input_size = np.prod(self.input_shape) * np.dtype(np.float32).itemsize
        self.output_size = np.prod(self.output_shape) * np.dtype(np.float32).itemsize
        
        self.d_input = self.cuda.malloc(self.input_size)
        self.d_output = self.cuda.malloc(self.output_size)
        
        # Set tensor addresses
        self.context.set_tensor_address(self.input_name, self.d_input)
        self.context.set_tensor_address(self.output_name, self.d_output)
    
    def __del__(self):
        """Cleanup GPU memory"""
        if hasattr(self, 'cuda'):
            if hasattr(self, 'd_input'):
                self.cuda.free(self.d_input)
            if hasattr(self, 'd_output'):
                self.cuda.free(self.d_output)
    
    def preprocess(self, image: np.ndarray) -> np.ndarray:
        """Preprocess image for YOLO model"""
        # Resize to 640x640
        img_resized = cv2.resize(image, (640, 640))
        
        # Convert BGR to RGB
        img_rgb = cv2.cvtColor(img_resized, cv2.COLOR_BGR2RGB)
        
        # Normalize to [0, 1]
        img_normalized = img_rgb.astype(np.float32) / 255.0
        
        # Change from HWC to CHW format
        img_chw = np.transpose(img_normalized, (2, 0, 1))
        
        # Add batch dimension: (1, 3, 640, 640)
        img_batch = np.expand_dims(img_chw, axis=0)
        
        return img_batch
    
    def infer(self, input_data: np.ndarray) -> np.ndarray:
        """Run inference with proper GPU memory management"""
        # Ensure input is contiguous and float32
        input_data = np.ascontiguousarray(input_data, dtype=np.float32)
        
        # Copy input to GPU
        self.cuda.memcpy_htod(
            self.d_input,
            input_data.ctypes.data,
            self.input_size
        )
        
        # Execute inference
        self.context.execute_async_v3(stream_handle=0)
        
        # Synchronize
        self.cuda.synchronize()
        
        # Allocate output array
        output = np.empty(self.output_shape, dtype=np.float32)
        
        # Copy output from GPU
        self.cuda.memcpy_dtoh(
            output.ctypes.data,
            self.d_output,
            self.output_size
        )
        
        return output
    
    def postprocess_yolo(self, output: np.ndarray, conf_threshold=0.25, iou_threshold=0.45, debug=False) -> List:
        """Post-process YOLO output to get detections"""
        # Output shape: (1, 27, 8400) -> reshape to (8400, 27)
        predictions = output[0].transpose((1, 0))  # (8400, 27)
        
        if debug:
            print(f"\n=== DEBUG INFO ===")
            print(f"Output shape: {output.shape}")
            print(f"Predictions shape: {predictions.shape}")
            print(f"Min/Max values in predictions: {predictions.min():.4f} / {predictions.max():.4f}")
            print(f"First few predictions:\n{predictions[:5]}")
        
        # Try different YOLO formats
        # Format 1: [x_center, y_center, w, h, objectness, class_scores...]
        # Format 2: [x1, y1, x2, y2, objectness, class_scores...] (already in corner format)
        # Format 3: [x_center, y_center, w, h, class_scores...] (no separate objectness)
        
        # Extract boxes (first 4 values)
        boxes = predictions[:, :4]
        objectness = predictions[:, 4:5]  # Objectness score
        class_scores = predictions[:, 5:]  # Class scores
        
        if debug:
            print(f"\nBoxes stats:")
            print(f"  Min/Max x: {boxes[:, 0].min():.4f} / {boxes[:, 0].max():.4f}")
            print(f"  Min/Max y: {boxes[:, 1].min():.4f} / {boxes[:, 1].max():.4f}")
            print(f"  Min/Max w: {boxes[:, 2].min():.4f} / {boxes[:, 2].max():.4f}")
            print(f"  Min/Max h: {boxes[:, 3].min():.4f} / {boxes[:, 3].max():.4f}")
            print(f"\nObjectness stats:")
            print(f"  Min/Max: {objectness.min():.4f} / {objectness.max():.4f}")
            print(f"  Mean: {objectness.mean():.4f}")
            print(f"  Values > 0.1: {(objectness > 0.1).sum()}")
            print(f"  Values > 0.25: {(objectness > 0.25).sum()}")
            print(f"  Values > 0.5: {(objectness > 0.5).sum()}")
            print(f"\nClass scores stats:")
            print(f"  Min/Max: {class_scores.min():.4f} / {class_scores.max():.4f}")
            print(f"  Mean: {class_scores.mean():.4f}")
        
        # Try different scoring methods
        # Method 1: objectness * class_scores (standard YOLO)
        scores_method1 = objectness * class_scores
        
        # Method 2: Use class_scores directly (if objectness is not reliable)
        scores_method2 = class_scores
        
        # Method 3: Apply sigmoid to objectness first (if it's logit)
        objectness_sigmoid = 1.0 / (1.0 + np.exp(-np.clip(objectness, -500, 500)))  # Clip to avoid overflow
        scores_method3 = objectness_sigmoid * class_scores
        
        # Choose method based on which gives better results
        # If objectness is too small, use method 2 or 3
        if objectness.max() < 0.01:
            if debug:
                print(f"\nObjectness too small, using class scores directly or sigmoid")
            # Try sigmoid first, fallback to direct class scores
            if objectness.max() > -10 and objectness.min() < 10:
                scores = scores_method3  # Use sigmoid
                if debug:
                    print(f"  Using sigmoid method")
            else:
                scores = scores_method2  # Use class scores directly
                if debug:
                    print(f"  Using class scores directly")
        else:
            scores = scores_method1  # Standard method
            if debug:
                print(f"  Using standard objectness * class_scores")
        
        # Get class IDs and max scores
        class_ids = np.argmax(scores, axis=1)
        max_scores = np.max(scores, axis=1)
        
        if debug:
            print(f"\nFinal scores stats:")
            print(f"  Min/Max: {max_scores.min():.4f} / {max_scores.max():.4f}")
            print(f"  Mean: {max_scores.mean():.4f}")
            print(f"  Values > 0.1: {(max_scores > 0.1).sum()}")
            print(f"  Values > 0.25: {(max_scores > 0.25).sum()}")
            print(f"  Values > 0.5: {(max_scores > 0.5).sum()}")
            print(f"  Using threshold: {conf_threshold}")
        
        # Filter by confidence
        valid = max_scores > conf_threshold
        boxes = boxes[valid]
        scores = max_scores[valid]
        class_ids = class_ids[valid]
        
        if debug:
            print(f"\nAfter filtering:")
            print(f"  Valid detections: {len(boxes)}")
        
        if len(boxes) == 0:
            if debug:
                print("No detections found!")
            return []
        
        # Check if boxes are already in corner format or center format
        # If max x > 1 or max y > 1, they might already be in pixel coordinates
        # If max x <= 1 and max y <= 1, they're normalized
        
        max_x = boxes[:, 0].max()
        max_y = boxes[:, 1].max()
        max_w = boxes[:, 2].max()
        max_h = boxes[:, 3].max()
        
        if debug:
            print(f"\nBox format check:")
            print(f"  Max x: {max_x:.4f}, Max y: {max_y:.4f}")
            print(f"  Max w: {max_w:.4f}, Max h: {max_h:.4f}")
        
        # Try center format first (most common for YOLO)
        # If coordinates are normalized (0-1), convert from center to corner
        if max_x <= 1.0 and max_y <= 1.0:
            # Normalized center format
            x_center, y_center, w, h = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
            x1 = (x_center - w / 2) * 640
            y1 = (y_center - h / 2) * 640
            x2 = (x_center + w / 2) * 640
            y2 = (y_center + h / 2) * 640
        elif max_x <= 640 and max_y <= 640:
            # Pixel center format
            x_center, y_center, w, h = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
            x1 = x_center - w / 2
            y1 = y_center - h / 2
            x2 = x_center + w / 2
            y2 = y_center + h / 2
        else:
            # Already in corner format (pixel coordinates)
            x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
        
        boxes_corners = np.stack([x1, y1, x2, y2], axis=1)
        
        # Clip to image bounds
        boxes_corners[:, 0] = np.clip(boxes_corners[:, 0], 0, 640)
        boxes_corners[:, 1] = np.clip(boxes_corners[:, 1], 0, 640)
        boxes_corners[:, 2] = np.clip(boxes_corners[:, 2], 0, 640)
        boxes_corners[:, 3] = np.clip(boxes_corners[:, 3], 0, 640)
        
        # Filter out invalid boxes (width or height <= 0)
        valid_boxes = (boxes_corners[:, 2] > boxes_corners[:, 0]) & (boxes_corners[:, 3] > boxes_corners[:, 1])
        boxes_corners = boxes_corners[valid_boxes]
        scores = scores[valid_boxes]
        class_ids = class_ids[valid_boxes]
        
        if debug:
            print(f"\nAfter clipping and validation:")
            print(f"  Valid boxes: {len(boxes_corners)}")
            if len(boxes_corners) > 0:
                print(f"  Sample box: {boxes_corners[0]}")
        
        if len(boxes_corners) == 0:
            return []
        
        # Apply NMS
        try:
            indices = cv2.dnn.NMSBoxes(
                boxes_corners.tolist(),
                scores.tolist(),
                conf_threshold,
                iou_threshold
            )
            
            if indices is not None and len(indices) > 0:
                indices = indices.flatten()
                if debug:
                    print(f"\nAfter NMS:")
                    print(f"  Detections: {len(indices)}")
                return [
                    {
                        'box': boxes_corners[i],
                        'score': float(scores[i]),
                        'class_id': int(class_ids[i])
                    }
                    for i in indices
                ]
        except Exception as e:
            if debug:
                print(f"NMS error: {e}")
            # Return all boxes if NMS fails
            return [
                {
                    'box': boxes_corners[i],
                    'score': float(scores[i]),
                    'class_id': int(class_ids[i])
                }
                for i in range(len(boxes_corners))
            ]
        
        return []
    
    def draw_detections(self, image: np.ndarray, detections: List, scale_to_original=True) -> np.ndarray:
        """Draw bounding boxes on image"""
        img_copy = image.copy()
        orig_h, orig_w = image.shape[:2]
        
        for det in detections:
            x1, y1, x2, y2 = det['box']
            score = det['score']
            class_id = int(det['class_id'])
            
            # Scale coordinates if image was resized
            if scale_to_original and (orig_w != 640 or orig_h != 640):
                scale_x = orig_w / 640.0
                scale_y = orig_h / 640.0
                x1 = int(x1 * scale_x)
                y1 = int(y1 * scale_y)
                x2 = int(x2 * scale_x)
                y2 = int(y2 * scale_y)
            else:
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            
            # Ensure coordinates are within image bounds
            x1 = max(0, min(x1, orig_w))
            y1 = max(0, min(y1, orig_h))
            x2 = max(0, min(x2, orig_w))
            y2 = max(0, min(y2, orig_h))
            
            # Draw rectangle
            cv2.rectangle(img_copy, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Draw label
            label = f"Class {class_id}: {score:.2f}"
            cv2.putText(img_copy, label, (x1, y1 - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        return img_copy


def test_single_image(image_path: str, engine_path: str = "model.engine", debug=True):
    """Test inference on a single image"""
    # Initialize inference engine
    inferencer = TensorRTInference(engine_path)
    
    # Load image
    image = cv2.imread(image_path)
    if image is None:
        print(f"Error: Could not load image {image_path}")
        return
    
    print(f"Original image shape: {image.shape}")
    
    # Preprocess
    input_data = inferencer.preprocess(image)
    print(f"Input shape: {input_data.shape}")
    
    # Run inference
    start_time = time.time()
    output = inferencer.infer(input_data)
    inference_time = time.time() - start_time
    print(f"Inference time: {inference_time*1000:.2f} ms")
    
    # Post-process with debug
    detections = inferencer.postprocess_yolo(output, conf_threshold=0.25, debug=debug)
    print(f"\nFound {len(detections)} detections")
    
    if len(detections) > 0:
        print(f"Sample detection: {detections[0]}")
    
    # Draw and save
    result_image = inferencer.draw_detections(image, detections)
    output_path = "result.jpg"
    cv2.imwrite(output_path, result_image)
    print(f"Result saved to {output_path}")
    
    return detections


def test_video(video_path: str, engine_path: str = "model.engine", output_path: str = "output_video.mp4"):
    """Test inference on a video"""
    # Initialize inference engine
    inferencer = TensorRTInference(engine_path)
    
    # Open video
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"Error: Could not open video {video_path}")
        return
    
    # Get video properties
    fps = int(cap.get(cv2.CAP_PROP_FPS))
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    
    # Create video writer
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(output_path, fourcc, fps, (width, height))
    
    frame_count = 0
    total_time = 0
    
    print("Processing video...")
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # Preprocess
        input_data = inferencer.preprocess(frame)
        
        # Run inference
        start_time = time.time()
        output = inferencer.infer(input_data)
        inference_time = time.time() - start_time
        total_time += inference_time
        
        # Post-process (lower threshold for video)
        detections = inferencer.postprocess_yolo(output, conf_threshold=0.25, debug=False)
        
        # Draw detections
        result_frame = inferencer.draw_detections(frame, detections)
        
        # Resize back to original if needed
        if result_frame.shape[:2] != (height, width):
            result_frame = cv2.resize(result_frame, (width, height))
        
        # Write frame
        out.write(result_frame)
        
        frame_count += 1
        if frame_count % 30 == 0:
            print(f"Processed {frame_count} frames, avg FPS: {frame_count/total_time:.2f}")
    
    cap.release()
    out.release()
    print(f"Video processing complete! Average FPS: {frame_count/total_time:.2f}")
    print(f"Output saved to {output_path}")


if __name__ == "__main__":
    import sys
    
    if len(sys.argv) < 2:
        print("Usage:")
        print("  python test_inference.py <image_path>")
        print("  python test_inference.py <video_path> --video")
        print("\nExample:")
        print("  python test_inference.py test.jpg")
        print("  python test_inference.py test.mp4 --video")
        sys.exit(1)
    
    file_path = sys.argv[1]
    engine_path = sys.argv[2] if len(sys.argv) > 2 and sys.argv[2] != "--video" else "model.engine"
    is_video = "--video" in sys.argv
    
    if is_video:
        test_video(file_path, engine_path)
    else:
        test_single_image(file_path, engine_path)