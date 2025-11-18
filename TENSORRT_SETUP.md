# TensorRT Setup Status ✅

## What You Have Installed

✅ **TensorRT Python**: Version 10.3.0 (working!)
✅ **TensorRT System Packages**: All installed via Debian packages
✅ **trtexec**: Available at `/usr/src/tensorrt/bin/trtexec`
✅ **CUDA**: Version 12.6 (detected via nvidia-smi)
✅ **GPU**: NVIDIA Orin (Jetson device)

## How to Use trtexec

Since `trtexec` is not in your PATH, you have two options:

### Option 1: Use Full Path (Quick)
```bash
/usr/src/tensorrt/bin/trtexec --onnx=model.onnx --saveEngine=engine.engine
```

### Option 2: Add to PATH (Recommended)
Add this to your `~/.bashrc`:
```bash
export PATH=$PATH:/usr/src/tensorrt/bin
```

Then reload:
```bash
source ~/.bashrc
```

Now you can use `trtexec` directly:
```bash
trtexec --onnx=model.onnx --saveEngine=engine.engine
```

## Next Steps Based on TensorRT Documentation

### Step 1: Export Your Model to ONNX
If you have a PyTorch model:
```python
import torch
import torchvision.models as models

# Load your model
model = models.resnet50(pretrained=True).eval()

# Export to ONNX
dummy_input = torch.randn(1, 3, 224, 224)
torch.onnx.export(model, dummy_input, "model.onnx", verbose=False)
```

Or download a test model:
```bash
wget https://download.onnxruntime.ai/onnx/models/resnet50.tar.gz
tar xzf resnet50.tar.gz
```

### Step 2: Select Precision
Choose your precision (FP32, FP16, INT8, etc.)
- **FP32**: Default, highest accuracy
- **FP16**: Faster, good accuracy
- **INT8**: Fastest, requires calibration

### Step 3: Convert ONNX to TensorRT Engine
```bash
# Using full path
/usr/src/tensorrt/bin/trtexec --onnx=model.onnx --saveEngine=model.engine

# Or if you added to PATH:
trtexec --onnx=model.onnx --saveEngine=model.engine

# With FP16 precision:
trtexec --onnx=model.onnx --saveEngine=model_fp16.engine --fp16

# With strong typing:
trtexec --onnx=model.onnx --saveEngine=model.engine --stronglyType
```

### Step 4: Deploy Using Python Runtime API
```python
import tensorrt as trt
import numpy as np

# Load engine
with open('model.engine', 'rb') as f:
    engine_data = f.read()

runtime = trt.Runtime(trt.Logger(trt.Logger.WARNING))
engine = runtime.deserialize_cuda_engine(engine_data)

# Create context
context = engine.create_execution_context()

# Allocate buffers and run inference
# (See TensorRT Python API documentation for full example)
```

## Quick Test

Test your setup with a sample model:
```bash
# Download test model
wget https://download.onnxruntime.ai/onnx/models/resnet50.tar.gz
tar xzf resnet50.tar.gz

# Convert to TensorRT engine
/usr/src/tensorrt/bin/trtexec --onnx=resnet50/model.onnx --saveEngine=resnet50.engine

# If successful, you'll see "Engine built successfully"
```

## Your Current Files

- `model_convert.py` - Use this to convert your models
- `frame_processor.py` - Use this to run inference with TensorRT engines

## Summary

✅ **You're all set!** TensorRT is fully installed and ready to use.
- Python API: ✅ Working
- trtexec: ✅ Available (use full path or add to PATH)
- CUDA/GPU: ✅ Detected

You can now proceed with converting and deploying your models!

