#!/bin/bash
# setup_container.sh
#
# Re-run this once, every time you start a FRESH container
# (since --rm containers lose everything installed outside /workspace).
#
# Usage (from inside a freshly started container):
#   cd /workspace
#   bash setup_container.sh
#
# Safe to re-run on an already-set-up container too -- apt/pip will just
# report things as already installed/satisfied.

set -e  # stop immediately if any step fails, instead of plowing ahead

echo "=================================================="
echo "1/6: Installing apt packages (Boost, OpenCV, cv_bridge,"
echo "     tf2_ros, build tools, Nav2, RTAB-Map, CycloneDDS RMW)..."
echo "=================================================="
apt update
apt install -y \
  libboost-all-dev \
  python3-opencv \
  ros-humble-cv-bridge \
  ros-humble-tf2-ros \
  cmake \
  build-essential \
  python3-pip \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-rtabmap-ros \
  ros-humble-rmw-cyclonedds-cpp

echo "=================================================="
echo "1b/6: Setting RMW_IMPLEMENTATION to match the robot's"
echo "      host-side LiDAR/odometry service (rmw_cyclonedds_cpp)."
echo "      Without this, topics like /utlidar/robot_odom will"
echo "      show up in 'ros2 topic list' but publish NO data when"
echo "      checked from inside this container."
echo "=================================================="
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
if ! grep -q "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" ~/.bashrc; then
  echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
fi

echo "=================================================="
echo "2/6: Setting up CycloneDDS (Python bindings for the SDK)..."
echo "=================================================="
export CYCLONEDDS_HOME=/workspace/cyclonedds/install

if [ ! -d "$CYCLONEDDS_HOME" ]; then
  echo "CycloneDDS C library not found at $CYCLONEDDS_HOME -- building from source"
  echo "(this only needs to happen once ever, since /workspace persists)"
  cd /workspace
  if [ ! -d "cyclonedds" ]; then
    git clone https://github.com/eclipse-cyclonedds/cyclonedds.git
  fi
  cd cyclonedds
  git fetch --tags
  git checkout 0.10.2
  mkdir -p build install
  cd build
  cmake -DCMAKE_INSTALL_PREFIX=../install ..
  cmake --build . --target install
  cd /workspace
else
  echo "CycloneDDS C library already present at $CYCLONEDDS_HOME -- skipping rebuild"
fi

echo "Installing/reinstalling cyclonedds Python bindings (version must match: 0.10.2)..."
pip3 install cyclonedds==0.10.2

echo "=================================================="
echo "3/6: Checking/fixing NumPy version (cv_bridge needs <2.0)..."
echo "=================================================="
NUMPY_VERSION=$(python3 -c "import numpy; print(numpy.__version__)")
echo "Current numpy version: $NUMPY_VERSION"
if [[ "$NUMPY_VERSION" == 2.* ]]; then
  echo "NumPy 2.x detected -- downgrading..."
  pip3 install "numpy<2" --force-reinstall
else
  echo "NumPy version OK."
fi

echo "=================================================="
echo "4/6: Persisting CYCLONEDDS_HOME for this and future shells in this container..."
echo "=================================================="
if ! grep -q "CYCLONEDDS_HOME=" ~/.bashrc; then
  echo 'export CYCLONEDDS_HOME=/workspace/cyclonedds/install' >> ~/.bashrc
fi

echo "=================================================="
echo "5/6: Building the workspace..."
echo "=================================================="
cd /workspace
source /opt/ros/humble/setup.bash
colcon build

echo "=================================================="
echo "6/6: Setup complete. To use the workspace in THIS shell, run:"
echo "  source /opt/ros/humble/setup.bash"
echo "  source /workspace/install/setup.bash"
echo ""
echo "New shells (e.g. via 'docker exec') opened from now on will have"
echo "CYCLONEDDS_HOME and RMW_IMPLEMENTATION set automatically via"
echo "~/.bashrc, but you still need to source the ROS/workspace setup"
echo "files each time."
echo "=================================================="
