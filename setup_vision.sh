#!/bin/bash
# setup_vision.sh -- go2vision container ONLY. Do not run this in go2nav.
#
# Purpose: GPU-accelerated YOLO detection/tracking node(s).
# Base image: dustynv/ros:humble-ros-base-l4t-r35.3.1 (Ubuntu 20.04 / Focal)
# GPU: required (--runtime nvidia).
#
# Wheel caching: torch and torchvision wheels are saved to /workspace/wheels/
# (the SHARED bind mount) so recreating this container never requires
# rebuilding them from scratch again -- only the first-ever run pays the
# ~30-60 min torchvision compile cost.
#
# Usage: cd /workspace && bash setup_vision.sh

set -e

# ---- SAFETY GUARD: refuse to run in the wrong container ----
OS_CODENAME=$(grep VERSION_CODENAME /etc/os-release | cut -d= -f2)
if [ "$OS_CODENAME" != "focal" ]; then
  echo "############################################################"
  echo "ABORT: this is setup_vision.sh, but the OS here is '$OS_CODENAME',"
  echo "not 'focal'. This script must only run inside go2vision."
  echo "You are very likely inside go2nav by mistake."
  echo "############################################################"
  exit 1
fi
if [ -f /nav_build/install/setup.bash ] || [ -d /nav_build ]; then
  echo "############################################################"
  echo "ABORT: found /nav_build in this container -- that mount belongs"
  echo "to go2nav only. Check your docker run command."
  echo "############################################################"
  exit 1
fi
if ! python3 -c "import ctypes; ctypes.CDLL('libnvinfer.so')" 2>/dev/null; then
  echo "############################################################"
  echo "ABORT: libnvinfer.so not found -- this container has no GPU/"
  echo "TensorRT access. Was it started with --runtime nvidia?"
  echo "############################################################"
  exit 1
fi

echo "################################################"
echo "# go2vision setup starting (guards passed: focal OS, no /nav_build,"
echo "# GPU/TensorRT confirmed)"
echo "################################################"

echo "== 1/7: GPU/TensorRT access confirmed above =="

echo "== 2/7: System libs (libopenblas for NVIDIA's torch wheel, libgomp"
echo "        preload to avoid the ARM64 static-TLS import crash) =="
apt update
apt install -y libopenblas-dev git cmake \
  libjpeg-dev zlib1g-dev libpython3-dev \
  libavcodec-dev libavformat-dev libswscale-dev

LIBGOMP_PATH=$(find / -name "libgomp.so.1" 2>/dev/null | head -n 1)
if [ -n "$LIBGOMP_PATH" ]; then
  export LD_PRELOAD=$LIBGOMP_PATH
fi

echo "== 3/7: NumPy (pin 1.24.4 -- highest 1.x published for Python 3.8;"
echo "        NVIDIA's own docs suggest 1.26.1 but that doesn't exist for cp38) =="
pip3 install numpy==1.24.4

echo "== 4/7: PyTorch -- NVIDIA's Jetson-specific wheel, NOT generic pip torch"
echo "        (generic PyPI torch has zero CUDA support on Jetson) =="
mkdir -p /workspace/wheels
TORCH_WHL=/workspace/wheels/torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl
if [ ! -f "$TORCH_WHL" ]; then
  wget -O "$TORCH_WHL" \
    https://developer.download.nvidia.com/compute/redist/jp/v511/pytorch/torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl
fi
pip3 install --no-cache --no-deps "$TORCH_WHL"

echo "== 5/7: torchvision -- built from source (tag v0.15.2, matches torch"
echo "        2.0.0), cached as a wheel in /workspace/wheels/ so this only"
echo "        happens once ever, not once per container recreation =="
TV_WHL=$(ls /workspace/wheels/torchvision-0.15.2*.whl 2>/dev/null | head -n 1)
if [ -z "$TV_WHL" ]; then
  echo "No cached torchvision wheel found -- building from source (this WILL"
  echo "take 30-60+ minutes on Jetson's CPU, this is expected)"
  cd /root
  if [ ! -d torchvision_src ]; then
    git clone --branch v0.15.2 https://github.com/pytorch/vision torchvision_src
  fi
  cd torchvision_src
  export BUILD_VERSION=0.15.2
  python3 setup.py bdist_wheel
  cp dist/torchvision-0.15.2*.whl /workspace/wheels/
  TV_WHL=$(ls /workspace/wheels/torchvision-0.15.2*.whl | head -n 1)
  cd /workspace
else
  echo "Found cached wheel: $TV_WHL -- skipping the 30-60 min rebuild entirely"
fi
# --no-deps is critical here: without it, pip silently reinstalls generic
# PyPI torch as a "dependency", clobbering the NVIDIA build we just installed.
pip3 install --no-deps --force-reinstall "$TV_WHL"

echo "== 6/7: ultralytics + export dependencies =="
pip3 install ultralytics
pip3 install "onnx>=1.12.0,<2.0.0" onnxslim onnxruntime || true

echo "== 7/7: Build workspace into SEPARATE /vision_build, then .bashrc =="
mkdir -p /vision_build
cd /workspace
source /opt/ros/humble/install/setup.bash
colcon build --build-base /vision_build/build --install-base /vision_build/install

sed -i '/RMW_IMPLEMENTATION/d;/CYCLONEDDS_HOME/d;/opt\/ros\/humble/d;/install\/setup.bash/d;/PS1=/d;/LD_PRELOAD/d' ~/.bashrc
cat >> ~/.bashrc << EOF
# --- go2vision specific, do not copy to go2nav ---
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_PRELOAD=$LIBGOMP_PATH
source /opt/ros/humble/install/setup.bash
source /vision_build/install/setup.bash
export PS1='\[\e[44m\][GO2VISION]\[\e[0m\] \w \$ '
EOF

echo "################################################"
echo "# go2vision setup complete."
echo "# Verify: python3 -c \"import torch,torchvision; print(torch.__version__, torchvision.__version__, torch.cuda.is_available())\""
echo "# Open a NEW shell (docker exec) to pick up .bashrc."
echo "################################################"
