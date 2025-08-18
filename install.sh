#!/bin/sh
# Installation script for Mobile System Control ROS 2 dependencies (Foxy/Humble).

# Exit immediately if a command exits with a non-zero status.
set -e

# --- Section 1: System and Python Dependencies ---
# Install basic system packages and Python libraries.
sudo apt-get update
sudo apt-get install -y python3-pip libeigen3-dev
pip3 install --user transforms3d networkx pygame

# --- Section 2: Core ROS 2 Tools and Setup ---
# Install and initialize essential ROS 2 tools like rosdep and colcon.
sudo apt-get install -y python3-rosdep python3-colcon-common-extensions
# Initialize rosdep (only needs to be done once on a system).
# If it has been run before, this may show an error, which is safe to ignore.
sudo rosdep init || echo "rosdep has already been initialized."
rosdep update

# --- Section 3: OSQP & OsqpEigen Solver Installation ---
# Clone, build, and install the OSQP and OsqpEigen libraries from source.
# A temporary directory will be used for the build process.
BUILD_DIR=$(mktemp -d)
cd $BUILD_DIR

# Clone and install OSQP
OSQP_repo="https://github.com/rise-lab-skku-racing/osqp.git"
git clone --recurse-submodules "$OSQP_repo" ./osqp
cd osqp
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
cd ../../

# Clone and install OsqpEigen
OSQP_eigen_repo="https://github.com/robotology/osqp-eigen.git"
git clone --recurse-submodules "$OSQP_eigen_repo" ./osqp-eigen
cd osqp-eigen
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
cd ../../

# Clean up the temporary build directory
rm -rf $BUILD_DIR


export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LC_NUMERIC="en_US.UTF-8”