#!/bin/bash
# setup_nav.sh -- go2nav container ONLY. Do not run this in go2vision.
#
# Purpose: Nav2 + RTAB-Map + explore_lite + Unitree SDK bridge nodes.
# Base image: arm64v8/ros:humble-ros-base (Ubuntu 22.04 / Jammy)
# GPU: none needed.
#
# Usage: cd /workspace && bash setup_nav.sh

set -e

# ---- SAFETY GUARD: refuse to run in the wrong container ----
OS_CODENAME=$(grep VERSION_CODENAME /etc/os-release | cut -d= -f2)
if [ "$OS_CODENAME" != "jammy" ]; then
  echo "############################################################"
  echo "ABORT: this is setup_nav.sh, but the OS here is '$OS_CODENAME',"
  echo "not 'jammy'. This script must only run inside go2nav."
  echo "You are very likely inside go2vision by mistake."
  echo "############################################################"
  exit 1
fi
if [ -f /vision_build/install/setup.bash ] || [ -d /vision_build ]; then
  echo "############################################################"
  echo "ABORT: found /vision_build in this container -- that mount"
  echo "belongs to go2vision only. Check your docker run command."
  echo "############################################################"
  exit 1
fi
echo "################################################"
echo "# go2nav setup starting"
echo "################################################"

echo "== 1/6: apt packages =="
apt update
apt install -y \
  libboost-all-dev \
  python3-opencv \
  ros-humble-cv-bridge \
  ros-humble-tf2-ros \
  cmake \
  build-essential \
  python3-pip \
  iproute2 \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-rtabmap-ros \
  ros-humble-rmw-cyclonedds-cpp

echo "== 2/6: CycloneDDS C library (built from source, pinned 0.10.2 to"
echo "        match unitree_sdk2py's Python bindings requirement) =="
export CYCLONEDDS_HOME=/workspace/cyclonedds/install
if [ ! -d "$CYCLONEDDS_HOME" ]; then
  cd /workspace
  git clone https://github.com/eclipse-cyclonedds/cyclonedds.git
  cd cyclonedds
  git fetch --tags
  git checkout 0.10.2
  mkdir -p build install
  cd build
  cmake -DCMAKE_INSTALL_PREFIX=../install ..
  cmake --build . --target install
  cd /workspace
else
  echo "Already present at $CYCLONEDDS_HOME -- skipping rebuild"
fi
pip3 install cyclonedds==0.10.2

echo "== 3/6: NumPy (<2.0 required by cv_bridge) =="
NUMPY_VERSION=$(python3 -c "import numpy; print(numpy.__version__)" 2>/dev/null || echo "none")
if [[ "$NUMPY_VERSION" == 2.* ]] || [[ "$NUMPY_VERSION" == "none" ]]; then
  pip3 install "numpy<2" --force-reinstall
fi

echo "== 4/6: Unitree SDK =="
if [ ! -d /workspace/sdk/unitree_sdk2_python ]; then
  mkdir -p /workspace/sdk
  git clone https://github.com/unitreerobotics/unitree_sdk2_python.git /workspace/sdk/unitree_sdk2_python
fi

echo "== 5/6: Build workspace into SEPARATE /nav_build (never /workspace/install --"
echo "        that directory is also written by go2vision under a different"
echo "        Python version and would silently corrupt shared packages) =="
mkdir -p /nav_build
cd /workspace
source /opt/ros/humble/setup.bash
colcon build --build-base /nav_build/build --install-base /nav_build/install

echo "== 6/6: .bashrc (go2nav-specific: cyclonedds RMW, standard ROS path) =="
sed -i '/RMW_IMPLEMENTATION/d;/CYCLONEDDS_HOME/d;/opt\/ros\/humble/d;/install\/setup.bash/d;/PS1=/d' ~/.bashrc
cat >> ~/.bashrc << 'EOF'
# --- go2nav specific, do not copy to go2vision ---
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_HOME=/workspace/cyclonedds/install
source /opt/ros/humble/setup.bash
source /nav_build/install/setup.bash
export PS1='\[\e[42m\][GO2NAV]\[\e[0m\] \w \$ '
EOF

echo "################################################"
echo "# go2nav setup complete."
echo "# Open a NEW shell (docker exec) to pick up .bashrc."
echo "#"
echo "# Reminder: go2_sport_bridge and cameraimg need the fastrtps"
echo "# override, since they touch the Unitree SDK's own CycloneDDS"
echo "# build directly and conflict with this container's cyclonedds RMW:"
echo "#   RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 run go2_nav go2_sport_bridge"
echo "#   RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 run go2_vision cameraimg"
echo "################################################"
