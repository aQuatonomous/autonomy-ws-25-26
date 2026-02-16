# Performance Analysis & Optimization Report

## Current Performance Issues

### 1. **GPU Underutilization (CRITICAL)**
- **Current**: GPU at **44% utilization** (should be 80-95%)
- **Root Cause**: CPU preprocessing bottleneck blocking GPU
- **Impact**: GPU is waiting for CPU, wasting compute resources

### 2. **Model Precision Issue (CRITICAL)**
- **Current**: Model is **FP32** (`DataType.FLOAT`)
- **Should Be**: **FP16** for ~2x speedup on Jetson
- **Impact**: Inference is 2x slower than it could be
- **Fix**: Rebuild engine with `--fp16` flag

### 3. **CPU Bottleneck**
- **Current**: All CPU cores at **74-80%** usage
- **Bottleneck**: Preprocessing operations:
  - `cv2.resize()` - CPU-bound
  - `cv2.cvtColor()` - CPU-bound  
  - Normalization/transpose - CPU-bound
- **Impact**: CPU can't keep up with GPU, causing stalls

### 4. **Staleness Issues**
- **Fixed**: Now publishing `detection_info` every frame
- **Fixed**: Increased staleness threshold to 2.0s
- **Status**: Should be resolved with optimizations

## Optimizations Applied

### ✅ Code Optimizations (Just Applied)
1. **Optimized Preprocessing**:
   - Use `INTER_AREA` for downscaling (faster)
   - Pre-allocate canvas as float32 (avoid conversion)
   - Use array slicing for BGR->RGB (faster than cvtColor)
   - Combine operations to reduce memory copies

2. **CUDA Streams**:
   - Added async memory copies (`memcpy_htod_async`, `memcpy_dtoh_async`)
   - Using CUDA streams for pipelining
   - Should reduce GPU idle time

3. **Detection Info Publishing**:
   - Now publishes every frame (not just inference frames)
   - Prevents combiner staleness warnings

### ⚠️ Still Needed (Critical)

#### 1. **Rebuild Model as FP16** (HIGHEST PRIORITY)
```bash
cd ~/autonomy-ws-25-26/computer_vision
bash cv_scripts/build_fp16_engine.sh
# OR manually:
/usr/src/tensorrt/bin/trtexec \
  --onnx=model_building_and_training/aqua_main.onnx \
  --saveEngine=cv_scripts/model.engine \
  --fp16 \
  --memPoolSize=workspace:4096 \
  --skipInference
```
**Expected Improvement**: ~2x faster inference (4 FPS → 8 FPS)

#### 2. **Consider GPU-Accelerated Preprocessing**
- Use CUDA-accelerated OpenCV operations (if available)
- Or use TensorRT's built-in preprocessing layers
- **Impact**: Could reduce CPU load significantly

#### 3. **Monitor Performance**
```bash
# Check GPU utilization
tegrastats --interval 1000

# Check CPU usage
htop

# Check inference FPS in logs
ros2 topic echo /camera0/detection_info | grep output_fps
```

## Expected Performance After FP16 Rebuild

- **Inference FPS**: 4 FPS → **8-10 FPS** (with inference_interval=2)
- **Display FPS**: Should maintain **~15 FPS**
- **GPU Utilization**: 44% → **80-90%**
- **CPU Usage**: Should decrease slightly
- **Staleness**: Should be eliminated

## Testing Commands

```bash
# Launch with optimized settings
cd ~/autonomy-ws-25-26/computer_vision
ros2 launch cv_ros_nodes launch_cv.py inference_interval:=2

# Monitor GPU
tegrastats --interval 1000

# Check detection FPS
ros2 topic hz /combined/detection_info

# Check for staleness warnings (should be minimal)
# Watch launch output for "Camera X is stale" warnings
```

## Next Steps

1. **IMMEDIATE**: Rebuild model as FP16 (biggest impact)
2. **Monitor**: Check GPU utilization after FP16 rebuild
3. **If still slow**: Consider GPU-accelerated preprocessing
4. **Fine-tune**: Adjust `inference_interval` based on actual performance
