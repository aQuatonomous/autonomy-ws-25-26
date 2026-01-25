# TensorRT Setup and Model Conversion

Machine-specific TensorRT usage, ONNX→engine conversion, and where artifacts live. For the full pipeline design (train → ONNX → engine → validate → deploy), see [DESIGN_STRATEGY.md](DESIGN_STRATEGY.md).

**Pipeline (summary):** `.pt` → `.onnx` (export) → `.engine` (trtexec) → `vision_inference` / launch.

---

## What is installed

- **TensorRT Python**: 10.3.0
- **trtexec**: `/usr/src/tensorrt/bin/trtexec`
- **CUDA**: 12.6 (nvidia-smi)
- **GPU**: NVIDIA Orin (Jetson)

---

## trtexec

### Option 1: Full path

```bash
/usr/src/tensorrt/bin/trtexec --onnx=model.onnx --saveEngine=model.engine
```

### Option 2: Add to PATH (recommended)

In `~/.bashrc`:

```bash
export PATH=$PATH:/usr/src/tensorrt/bin
```

Then `source ~/.bashrc` and run:

```bash
trtexec --onnx=model.onnx --saveEngine=model.engine
```

---

## ONNX → TensorRT engine

```bash
# FP32 (default)
trtexec --onnx=weights.onnx --saveEngine=model.engine --workspace=4096

# FP16 (faster, good accuracy on Jetson)
trtexec --onnx=weights.onnx --saveEngine=model_fp16.engine --fp16 --workspace=4096

# INT8 (fastest; requires calibration)
trtexec --onnx=weights.onnx --saveEngine=model_int8.engine --int8 --workspace=4096
```

**Precision:** FP32 = highest accuracy; FP16 = ~2× faster, &lt;1% loss; INT8 = ~4× faster, needs calibration.

---

## Python runtime (load and run engine)

```python
import tensorrt as trt

with open('model.engine', 'rb') as f:
    engine_data = f.read()

runtime = trt.Runtime(trt.Logger(trt.Logger.WARNING))
engine = runtime.deserialize_cuda_engine(engine_data)
context = engine.create_execution_context()
# Allocate buffers and run inference (see TensorRT Python API)
```

---

## Where things live

| Path | Role |
|------|------|
| `model_training/weights.pt` | Trained PyTorch weights |
| `model_training/weights.onnx` | ONNX (export from .pt, e.g. `torch.onnx.export`) |
| `model_training/test_inference.py` | TensorRT inference and validation on images/video |
| `model_training/test_onnx_single_image.py` | ONNX validation |
| `cv_scripts/model.engine` | TensorRT engine used by `vision_inference` |
| `vision_inference --engine_path`, launch `engine_path:=` | Override engine path |

---

## Engine specifications (YOLO)

- **Input:** `[1, 3, 640, 640]`, `float32`, CHW, values in [0, 1]
- **Output:** `[1, 27, 8400]` (example), `float32`, detection predictions

---

## Quick test

```bash
# Example with a downloaded ONNX
wget https://download.onnxruntime.ai/onnx/models/resnet50.tar.gz
tar xzf resnet50.tar.gz
/usr/src/tensorrt/bin/trtexec --onnx=resnet50/model.onnx --saveEngine=resnet50.engine
# "Engine built successfully" if okay
```

For YOLO: use `model_training/weights.onnx`, build engine, then run `model_training/test_inference.py` or `vision_inference` with the resulting `.engine`.
