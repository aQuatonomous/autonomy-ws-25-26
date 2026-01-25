"""
TensorRT YOLO Inference ROS2 Node

Subscribes to: /camera{N}/image_preprocessed
Publishes to: /camera{N}/detections (Image with bounding boxes)
              /camera{N}/detection_info (String with detection details)

Usage: python3 vision_inference.py --camera_id 0
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import tensorrt as trt
import numpy as np
from typing import List
import ctypes
import time
import json
import argparse


# ============================================================================
# CUDA MEMORY MANAGEMENT
# ============================================================================
class CudaRuntime:
    def __init__(self):
        try:
            self.cuda = ctypes.CDLL("libcudart.so")
        except:
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


# ============================================================================
# TENSORRT INFERENCE ENGINE
# ============================================================================
class TensorRTInference:
    def __init__(self, engine_path: str):
        """Load the TensorRT model file"""
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
        """Prepare image for the model"""
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
        """Run model inference on GPU"""
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
    
    def postprocess_yolo(self, output: np.ndarray, conf_threshold=0.25, iou_threshold=0.45) -> List:
        """Convert raw model output into bounding boxes"""
        # Output shape: (1, 27, 8400) -> reshape to (8400, 27)
        predictions = output[0].transpose((1, 0))
        
        # Extract boxes and scores
        boxes = predictions[:, :4]
        objectness = predictions[:, 4:5]
        class_scores = predictions[:, 5:]
        
        # Calculate final scores
        if objectness.max() < 0.01:
            objectness_sigmoid = 1.0 / (1.0 + np.exp(-np.clip(objectness, -500, 500)))
            scores = objectness_sigmoid * class_scores
        else:
            scores = objectness * class_scores
        
        # Get class IDs and max scores
        class_ids = np.argmax(scores, axis=1)
        max_scores = np.max(scores, axis=1)
        
        # Filter by confidence
        valid = max_scores > conf_threshold
        boxes = boxes[valid]
        scores = max_scores[valid]
        class_ids = class_ids[valid]
        
        if len(boxes) == 0:
            return []
        
        # Convert boxes to corner format
        max_x = boxes[:, 0].max()
        max_y = boxes[:, 1].max()
        
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
            # Already in corner format
            x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
        
        boxes_corners = np.stack([x1, y1, x2, y2], axis=1)
        
        # Clip to image bounds
        boxes_corners = np.clip(boxes_corners, 0, 640)
        
        # Filter invalid boxes
        valid_boxes = (boxes_corners[:, 2] > boxes_corners[:, 0]) & (boxes_corners[:, 3] > boxes_corners[:, 1])
        boxes_corners = boxes_corners[valid_boxes]
        scores = scores[valid_boxes]
        class_ids = class_ids[valid_boxes]
        
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
                return [
                    {
                        'box': boxes_corners[i],
                        'score': float(scores[i]),
                        'class_id': int(class_ids[i])
                    }
                    for i in indices
                ]
        except Exception as e:
            print(f"NMS error: {e}")
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


# ============================================================================
# ROS2 INFERENCE NODE
# ============================================================================
class InferenceNode(Node):
    
    def __init__(self, camera_id: int, engine_path: str = "model.engine", conf_threshold: float = 0.25):
        super().__init__(f'inference_node_camera{camera_id}')
        self.camera_id = camera_id
        
        # Initialize TensorRT engine
        self.get_logger().info(f'Loading TensorRT engine from {engine_path}...')
        self.inferencer = TensorRTInference(engine_path)
        self.conf_threshold = conf_threshold
        self.get_logger().info('TensorRT engine loaded successfully!')
        
        # ROS2 components
        self.bridge = CvBridge()
        self.frame_count = 0
        self.total_inference_time = 0
        
        # Subscribe to preprocessed images
        self.subscription = self.create_subscription(
            Image,
            f'/camera{camera_id}/image_preprocessed',
            self.image_callback,
            10
        )
        
        # Publisher for images with bounding boxes
        self.detection_image_pub = self.create_publisher(
            Image,
            f'/camera{camera_id}/detections',
            10
        )
        
        # Publisher for detection information (JSON)
        self.detection_info_pub = self.create_publisher(
            String,
            f'/camera{camera_id}/detection_info',
            10
        )
        
        self.get_logger().info(f'Inference node for camera{camera_id} ready!')
        self.get_logger().info(f'Subscribed to: /camera{camera_id}/image_preprocessed')
        self.get_logger().info(f'Publishing to: /camera{camera_id}/detections, /camera{camera_id}/detection_info')
    
    def image_callback(self, msg: Image):
        """Process incoming images and run inference"""
        try:
            # Convert ROS2 Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            orig_h, orig_w = cv_image.shape[:2]
            
            # Preprocess (resize to 640x640 only for the model)
            input_data = self.inferencer.preprocess(cv_image)
            
            # Run inference
            start_time = time.time()
            output = self.inferencer.infer(input_data)
            inference_time = time.time() - start_time
            
            # Post-process
            detections = self.inferencer.postprocess_yolo(
                output,
                conf_threshold=self.conf_threshold
            )
            
            # Update statistics
            self.frame_count += 1
            self.total_inference_time += inference_time
            
            # Draw detections
            result_image = self.inferencer.draw_detections(cv_image, detections)
            
            # Add FPS info to image
            fps = 1.0 / inference_time if inference_time > 0 else 0
            avg_fps = self.frame_count / self.total_inference_time if self.total_inference_time > 0 else 0
            cv2.putText(result_image, f"FPS: {fps:.1f} | Avg: {avg_fps:.1f} | Det: {len(detections)}",
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Publish detection image
            detection_msg = self.bridge.cv2_to_imgmsg(result_image, encoding='bgr8')
            detection_msg.header = msg.header
            self.detection_image_pub.publish(detection_msg)
            
            # Publish detection info as JSON (bbox scaled from 640x640 to preprocessed frame)
            scale_x = orig_w / 640.0
            scale_y = orig_h / 640.0
            detection_info = {
                'camera_id': self.camera_id,
                'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                'frame_id': msg.header.frame_id,
                'num_detections': len(detections),
                'inference_time_ms': inference_time * 1000,
                'fps': fps,
                'frame_width': orig_w,
                'frame_height': orig_h,
                'detections': []
            }
            
            # Extract detailed information for each detection (scale bbox 640x640 -> orig size)
            for det in detections:
                x1, y1, x2, y2 = det['box']
                x1 = float(x1) * scale_x
                y1 = float(y1) * scale_y
                x2 = float(x2) * scale_x
                y2 = float(y2) * scale_y
                width = float(x2 - x1)
                height = float(y2 - y1)
                bbox_scaled = [x1, y1, x2, y2]
                
                detection_info['detections'].append({
                    'class_id': int(det['class_id']),
                    'score': float(det['score']),
                    'x1': x1,
                    'y1': y1,
                    'x2': x2,
                    'y2': y2,
                    'width': width,
                    'height': height,
                    'bbox': bbox_scaled
                })
            info_msg = String()
            info_msg.data = json.dumps(detection_info)
            self.detection_info_pub.publish(info_msg)
            
            # Log every 15 frames (~1 s at 15 fps)
            if self.frame_count % 15 == 0:
                self.get_logger().info(
                    f'Processed {self.frame_count} frames | '
                    f'Avg FPS: {avg_fps:.2f} | '
                    f'Detections: {len(detections)}'
                )
        
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='ROS2 TensorRT Inference Node')
    parser.add_argument('--camera_id', type=int, default=0,
                       help='Camera ID (0, 1, or 2)')
    parser.add_argument('--engine_path', type=str, default='model.engine',
                       help='Path to TensorRT engine file')
    parser.add_argument('--conf_threshold', type=float, default=0.25,
                       help='Confidence threshold (0.0-1.0)')
    args_parsed, _ = parser.parse_known_args(args=args)
    
    if args_parsed.camera_id not in [0, 1, 2]:
        print("Error: camera_id must be 0, 1, or 2")
        return
    
    node = InferenceNode(
        camera_id=args_parsed.camera_id,
        engine_path=args_parsed.engine_path,
        conf_threshold=args_parsed.conf_threshold
    )
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()