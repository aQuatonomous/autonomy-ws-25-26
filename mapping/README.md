# mapping-25-26

ROS 2 perception pipeline for autonomous surface vehicle navigation using LiDAR-based buoy detection and tracking.

## Overview

This repository implements a multi-stage perception system that processes LiDAR point clouds to detect and track navigation buoys. The pipeline integrates with computer vision and planning systems to provide reliable landmark detection for autonomous navigation.

## Core Components

### 1. LiDAR Range Filter (`lidar_range_filter`)
Preprocesses raw LiDAR point clouds with spatial filtering and temporal accumulation.

- **Filtering**: Range-based (horizontal distance) and height-based (z-axis) clipping
- **Transform**: Optional TF2 transformation to `base_link` frame
- **Buffering**: Time-windowed cloud storage with service-based retrieval
- **Accumulation**: Temporal point cloud merging for improved density

### 2. Buoy Detector (`buoy_detector`)
Detects buoy candidates using **DBSCAN clustering** with a hybrid Cartesian-polar approach.

**Algorithm**:
- Clusters points in Cartesian (x, y) space using Euclidean distance
- Computes cluster centroids and converts to polar coordinates (range, bearing)
- Validates clusters by size, point count, and geometric extent
- Outputs detections with confidence scores

**Key Parameters**:
- `eps`: DBSCAN distance threshold (meters) — primary tuning parameter
- `min_samples`: Minimum points per cluster
- `min/max_lateral_extent`: Size-based filtering

### 3. Buoy Tracker (`buoy_tracker`)
Maintains persistent buoy identities across frames using data association.

**Features**:
- Nearest-neighbor association with 3D Euclidean distance
- Exponential smoothing for position estimates
- Observation-count filtering to reduce false positives
- Automatic removal of lost tracks after consecutive misses
- Supports future integration with vision system for color classification

## Integration Points

### Computer Vision
- **Input**: Camera-based buoy detections with bearing hypotheses
- **Fusion**: LiDAR verifies range and validates visual detections
- **Output**: Tracked buoys with stable IDs for color/class annotation

### Planning
- **Localization**: Integrates with GPS/IMU fusion (MAVROS + `robot_localization`)
- **Mapping**: Converts detections to global map frame via TF2
- **Navigation**: Publishes buoy positions in both map and base_link frames

## Pipeline Architecture

```
LiDAR Driver → Range Filter → Buoy Detector → Buoy Tracker → Map Frame Projection
                    ↓              ↓               ↓
              /points_filtered  /buoy_detections  /tracked_buoys
```

## Custom Messages

- `BuoyDetection`: Raw detection with range, bearing, confidence, and geometric properties
- `BuoyDetectionArray`: Array of detections from DBSCAN
- `TrackedBuoy`: Persistent buoy with ID, position (Cartesian + polar), observation count
- `TrackedBuoyArray`: Array of tracked buoys

## Launch

```bash
# LiDAR preprocessing
ros2 launch pointcloud_filters lidar_range_filter.launch.py

# Buoy detection
ros2 launch pointcloud_filters buoy_detector.launch.py
```

## Dependencies

- ROS 2 (Humble/Iron)
- `sensor_msgs_py` (PointCloud2 utilities)
- `scikit-learn` (DBSCAN implementation)
- `tf2_ros` (coordinate transformations)
- Unitree LiDAR SDK (vendor driver)

## Notes

- DBSCAN clustering in Cartesian space ensures physically meaningful distance thresholds
- Polar output format matches navigation system requirements
- Thread-safe buffering with automatic eviction of stale data
- Designed for water environments with sparse returns and reflections
