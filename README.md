# neoslam

This package is a bio-inspired SLAM system for ROS 2 (tested on ROS 2 Jazzy) that combines deep learning visual features, Hierarchical Temporal Memory (HTM), and topological mapping for robust simultaneous localization and mapping.

## System Overview

![neoslam Architecture](neoslam_arch.png)

neoslam implements a complete visual SLAM pipeline with the following components:

- **Visual Feature Extractor**: Extracts deep features from camera images using AlexNet (PyTorch)
- **Binary Projector**: Reduces dimensionality using Locality-Sensitive Binary Hashing (LSBH)
- **Neocortex (HTM)**: Learns temporal sequences using Hierarchical Temporal Memory
- **Spatial View Cells**: Performs visual place recognition and loop closure detection
- **Pose Cells**: Maintains pose estimation through continuous attractor network dynamics
- **Experience Map**: Builds and maintains a topological map with iterative refinement

## Dependencies

In addition to standard ROS 2 dependencies, this package requires:

### ROS 2 Packages
- `rclcpp`
- `std_msgs`
- `sensor_msgs`
- `geometry_msgs`
- `nav_msgs`
- `visualization_msgs`
- `tf2_ros`
- `tf2_geometry_msgs`
- `cv_bridge`
- `image_transport`
- `topological_msgs` (custom package - must be installed separately)

### System Libraries
- `OpenCV` (for image processing)
- `Eigen3` (for matrix operations)
- `Boost` (serialization component for HTM)
- `Irrlicht` (optional, for 3D visualization)
- `OpenGL` (optional, for visualization)
- `PyTorch` with CUDA (for deep learning feature extraction)
- `PyBind11` (for Python-C++ integration)
- `Roaring Bitmaps` (for efficient sparse representation)

## Installation

### 1. Install System Dependencies

Install the base packages and tools required by the system, including Python Virtual Environment support:

```bash
sudo apt update
sudo apt install -y \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-image-transport-plugins \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-vision-opencv \
    libopencv-dev \
    libboost-all-dev \
    libirrlicht-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    libeigen3-dev \
    python3-opencv \
    python3-pip \
    python3-venv \
    python3-full \
    python3-numpy \
    python3-pybind11
```

### 2. Configure Python Virtual Environment (PEP 668)

Ubuntu 24.04 enforces externally managed Python environments. To install PyTorch and maintain ROS 2 compatibility, create a virtual environment that links to system packages:

```bash
# Create the environment outside the workspace
python3 -m venv --system-site-packages ~/ros2_env

# Activate the environment
source ~/ros2_env/bin/activate
```

### 3. Install PyTorch with GPU (CUDA) Support

With the virtual environment activated, install PyTorch optimized for NVIDIA GPUs:

```bash
pip3 install torch torchvision
```

*(Optional) To automate environment activation on new terminals, add this to your `~/.bashrc`:*
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_env/bin/activate
```

### 4. Clone Required Repositories

```bash
cd ~/ros2_jazzy_ws/src
git clone https://github.com/BorgesJVT/neoslam.git
git clone https://github.com/BorgesJVT/topological_msgs.git
```

### 5. Generate Random Projection Matrix

The binary projector requires a random projection matrix. Generate it with:

```bash
cd ~/ros2_jazzy_ws/src/neoslam/src/dim_reduction_and_binarization/random_matrix
python3 generate_random_matrix.py --rows 64896 --cols 1024 --output randomMatrix.bin
```

### 6. Build the Workspace

Ensure your virtual environment is active before running the build command:

```bash
cd ~/ros2_jazzy_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select topological_msgs neoslam --symlink-install
```

### 7. Source the Workspace

```bash
source ~/ros2_jazzy_ws/install/setup.bash
```

## Usage

### Basic Launch

neoslam provides launch files for different datasets:

```bash
# For iratAUS dataset
ros2 launch neoslam irataus.launch.py use_sim_time:=true

# For Robotarium dataset
# ros2 launch neoslam robotarium.launch.py use_sim_time:=true
```

### Playing Dataset Bags

In a separate terminal, play your ROS 2 bag file:

```bash
# For iratAUS dataset
ros2 bag play data/irat_aus_28112011.db3 --rate 1.0 --clock --start-paused

# Adjust rate as needed (1.0 = real-time, 2.0 = 2x speed, etc.)
```
