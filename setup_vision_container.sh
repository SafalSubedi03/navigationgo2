#!/bin/bash
# setup_vision_container.sh
#
# Minimal setup for the go2vision container (Focal-based, dustynv image,
# GPU/TensorRT already baked in via the base image). This container's only
# job is running YOLO inference on /go2/camera/compressed -- it does NOT
# need Nav2, RTAB-Map, or the CycloneDDS RMW rebuild that go2nav needs.
#
# Usage (from inside a freshly started go2vision container):
#   cd /workspace
#   bash setup_vision_container.sh

set -e

echo "=================================================="
echo "1/6: Confirming GPU/TensorRT access..."
echo "=================================================="
python3 -c "import ctypes; ctypes.CDLL('libnvinfer.so'); print('TensorRT lib found - OK')"

echo "=================================================="
echo "2/6: Installing system libs needed by PyTorch on Jetson"
echo "     (libgomp static-TLS issue, libopenblas for NVIDIA's torch wheel)..."
echo "=================================================="
apt update
apt install -y libopenblas-dev

LIBGOMP_PATH=$(find / -name "libgomp.so.1" 2>/dev/null | head -n 1)
if [ -z "$LIBGOMP_PATH" ]; then
  echo "WARNING: libgomp.so.1 not found -- torch import may fail with a"
  echo "static TLS block error. Install libgomp1 and re-run if that happens:"
  echo "  apt install -y libgomp1"
else
  echo "Found libgomp at: $LIBGOMP_PATH"
  if ! grep -q "LD_PRELOAD=$LIBGOMP_PATH" ~/.bashrc; then
    echo "export LD_PRELOAD=$LIBGOMP_PATH" >> ~/.bashrc
  fi
  export LD_PRELOAD=$LIBGOMP_PATH
fi

echo "=================================================="
echo "3/6: Pinning NumPy to a version available for Python 3.8..."
echo "     (1.26.1 from NVIDIA's docs isn't published for cp38 -- 1.24.4"
echo "     is the highest 1.x version available here and works fine)"
echo "=================================================="
pip3 install numpy==1.24.4

echo "=================================================="
echo "4/6: Installing NVIDIA's Jetson-specific PyTorch build"
echo "     (matches JetPack 5.1.1 / L4T R35.3.1, Python 3.8, CUDA-enabled --"
echo "     the generic 'pip install torch' from PyPI has NO CUDA support"
echo "     on Jetson and must not be used)"
echo "=================================================="
pip3 uninstall torch torchvision -y || true

if [ ! -f /workspace/torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl ]; then
  wget -O /workspace/torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl \
    https://developer.download.nvidia.com/compute/redist/jp/v511/pytorch/torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl
fi
pip3 install --no-cache /workspace/torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl

echo "=================================================="
echo "5/6: Installing ultralytics (YOLO)..."
echo "     NOTE: this will complain that torchvision is missing --"
echo "     that's expected for now. torchvision has no easy prebuilt wheel"
echo "     for this exact JetPack/torch combo and would need building from"
echo "     source. Only revisit this if a specific ultralytics feature"
echo "     actually fails at runtime because of it."
echo "=================================================="
pip3 install ultralytics || true

echo "=================================================="
echo "6/6: Verifying torch + CUDA..."
echo "=================================================="
python3 -c "
import torch
print('torch version:', torch.__version__)
print('CUDA available:', torch.cuda.is_available())
"

echo "=================================================="
echo "Setup complete. To use the workspace in a new shell, run:"
echo "  source /opt/ros/humble/setup.bash"
echo "  source /workspace/install/setup.bash"
echo ""
echo "LD_PRELOAD is now set automatically via ~/.bashrc for new shells."
echo "This container shares /workspace with go2nav via the same bind mount --"
echo "no separate git pull or file copying needed between them."
echo "=================================================="
