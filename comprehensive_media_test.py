#!/usr/bin/env python3
"""
Simple Media Testing Script for RoboBoat Buoy Detection Model
Tests images and videos with clear file naming
"""

import os
import cv2
import numpy as np
import time
from ultralytics import YOLO
from pathlib import Path
import argparse
import datetime

class SimpleMediaTester:
    def __init__(self, model_path, testing_media_dir):
        self.model = YOLO(model_path)
        self.testing_media_dir = Path(testing_media_dir)
        
        # Create unique run name with timestamp
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.run_name = f"test_{timestamp}"
        
        # Test configurations
        self.resolutions = [1080, 720, 480]
        
    def resize_image(self, image, target_height):
        """Resize image maintaining aspect ratio"""
        h, w = image.shape[:2]
        aspect_ratio = w / h
        new_width = int(target_height * aspect_ratio)
        return cv2.resize(image, (new_width, target_height))
    
    def test_image(self, image_path):
        """Test single image at multiple resolutions"""
        print(f"\n🖼️  Testing Image: {image_path.name}")
        
        # Load original image
        original_image = cv2.imread(str(image_path))
        if original_image is None:
            print(f"❌ Failed to load image: {image_path}")
            return
            
        for resolution in self.resolutions:
            print(f"🔍 Processing {resolution}p...")
            
            # Resize image
            resized_image = self.resize_image(original_image, resolution)
            
            # Run inference and save to a unique directory per image+resolution
            results = self.model.predict(
                source=resized_image,
                device=0,
                imgsz=640,
                conf=0.6,
                iou=0.5,
                max_det=30,
                verbose=False,
                save=True,
                project="runs/detect",
                name=f"{self.run_name}_image_{image_path.stem}_{resolution}p",
                exist_ok=False
            )
            
            print(f"✅ Saved: runs/detect/{self.run_name}_image_{image_path.stem}_{resolution}p/")
    
    def test_video(self, video_path):
        """Test single video at multiple resolutions"""
        print(f"\n🎬 Testing Video: {video_path.name}")
        
        for resolution in self.resolutions:
            print(f"🔍 Processing {resolution}p...")
            
            # Use streaming mode and iterate to trigger processing and saving
            for _ in self.model.predict(
                source=str(video_path),
                device=0,
                imgsz=640,
                conf=0.6,
                iou=0.5,
                max_det=30,
                verbose=False,
                save=True,
                project="runs/detect",
                name=f"{self.run_name}_video_{video_path.stem}_{resolution}p",
                exist_ok=True,
                vid_stride=1,
                workers=0,
            ):
                pass

            print(f"✅ Saved: runs/detect/{self.run_name}_video_{video_path.stem}_{resolution}p/")
    
    def run_all_tests(self):
        """Run tests on all media files"""
        print("🚀 Starting Simple Media Testing")
        print("=" * 50)
        
        # Test images
        image_files = list(self.testing_media_dir.glob("*.png"))
        if image_files:
            print(f"\n📸 Found {len(image_files)} image files")
            for image_file in image_files:
                self.test_image(image_file)
        
        # Test videos
        video_files = list(self.testing_media_dir.glob("*.mp4"))
        if video_files:
            print(f"\n🎥 Found {len(video_files)} video files")
            for video_file in video_files:
                self.test_video(video_file)
        
        print(f"\n✅ All tests completed! Check runs/detect/{self.run_name}_*/")

def main():
    parser = argparse.ArgumentParser(description='Simple Media Testing for Buoy Detection')
    parser.add_argument('--model', default='/home/lorenzo/computer_vision/weights.pt', 
                       help='Path to model weights')
    parser.add_argument('--media_dir', default='/home/lorenzo/computer_vision/testing_media', 
                       help='Path to testing media directory')
    
    args = parser.parse_args()
    
    # Initialize tester
    tester = SimpleMediaTester(args.model, args.media_dir)
    
    # Run all tests
    tester.run_all_tests()

if __name__ == "__main__":
    main()
