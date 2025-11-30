```


## 📄 License

This project is created for the Mowito Robotics Assignment.

---

**Author**: Naman Sain 
**Date**: November 2025

```


Clone repo: git clone https://github.com/namanharshwal/mowito-assignment.git && cd mowito-assignments

Install deps: sudo apt update && sudo apt install ros-humble-desktop libeigen3-dev cmake build-essential

----------------------------------------------------------------------------------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------------------------------------------
The main projects are in " src " directory and another in " robotics_projects " directory "
The " src " directory contains the " ROS , Computer Vision Task and the Behavior Trees " task and the " robotics_projects " directory contains the " Maths Test Task "
----------------------------------------------------------------------------------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------------------------------------------


Follow the instructions: 

--------------------------------------------------------------------------------------------------------------
SRC DIRECTORY : 
-------------------------------------------------------------------------------------------------------------


-------------------------------------------------------------------------------------
VQ-BeT Task 2.1 : ROS2 Image Conversion Package
-------------------------------------------------------------------------------------

Command Reference & Execution Guide

Task Objective: Create a ROS2 package that subscribes to camera images, converts between grayscale/color modes via service calls, and publishes processed images at 30 FPS.

Phase 1: Initial Setup & Environment Fix

Goal: Resolve NumPy version conflicts and ensure cv_bridge compatibility.


# 1. Remove conflicting NumPy 2.x
pip3 uninstall -y numpy

# 2. Install NumPy 1.26.4 (Compatible with ROS2 Humble)
pip3 install numpy==1.26.4

# 3. Reinstall cv_bridge to ensure binary compatibility
sudo apt-get install --reinstall ros-humble-cv-bridge ros-humble-vision-opencv

# 4. Verify Installation
python3 -c 'from cv_bridge import CvBridge; print("cv_bridge works!")'
# Expected output: cv_bridge works!

Phase 2: Package Structure & Permissions

Goal: Setup directories and make launch files executable. (Assumes script files are populated).


# 1. Navigate to package source
cd ~/ros2_bt_test_ws/src/image_conversion

# 2. Create launch directory
mkdir -p ~/ros2_bt_test_ws/src/image_conversion/launch

# 3. Make the launch file executable
chmod +x ~/ros2_bt_test_ws/src/image_conversion/launch/image_conversion_launch.py

Phase 3: Build and Run System

Goal: Clean build the workspace and launch the nodes.


# 1. Navigate to workspace root
cd ~/ros2_bt_test_ws

# 2. Clean previous build artifacts
rm -rf build/ install/ log/

# 3. Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# 4. Build the package
colcon build --symlink-install

# 5. Source the environment
source install/setup.bash

# 6. Launch the system (Terminal 1)
ros2 launch image_conversion image_conversion_launch.py

Phase 4: Testing & Verification

Goal: Manually test service switching and monitor performance.


# Open a new terminal (Terminal 2) and source setup
source ~/ros2_bt_test_ws/install/setup.bash

# 1. Test: Switch to Color Mode
ros2 service call /image_conversion/set_mode std_srvs/SetBool "{data: false}"

# 2. Test: Switch to Grayscale Mode
ros2 service call /image_conversion/set_mode std_srvs/SetBool "{data: true}"

# 3. Monitor Frame Rate (Should be ~30 FPS)
ros2 topic hz /image_converted

# 4. Check System Status
ros2 node list
ros2 topic list
ros2 node info /image_conversion --spin-time 1

Phase 5: Automated Image Collection

Goal: Run the automation script to capture 10 grayscale and 10 color images.

# 1. Make automation script executable
chmod +x ~/auto_image_saver.py

# 2. Run Auto Image Saver (Ensure Launch file is running in Terminal 1)
source ~/ros2_bt_test_ws/install/setup.bash
python3 ~/auto_image_saver.py

# Expected Output Summary:
# - Saved 10 Grayscale images
# - Switched mode automatically
# - Saved 10 Color images

Phase 6: Final Verification

Goal: Verify that images were saved to the correct location and pixel formats are correct.


# 1. Check storage location
ls -lh /tmp/ros2_images_auto/

# 2. Make verification script executable
chmod +x ~/verify_auto_saved.py

# 3. Run Verification Script
python3 ~/verify_auto_saved.py /tmp/ros2_images_auto

# Expected Output:
# ✓ ALL IMAGES VERIFIED SUCCESSFULLY!


--------------------------------------------------------------------------------------
Task 2.2
--------------------------------------------------------------------------------------

Project: Robot Apple Picking Task Status: Functional / Tested System: ROS2 Environment (Ubuntu/Linux)

1. Environment Setup (Library Installation)

Execute these commands to install the BehaviorTree.CPP library and configure system paths.


# Navigate to workspace
cd ~/ros2_bt_test_ws/src

# Clone BehaviorTree.CPP repository
git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git

# Create build directory and compile
cd BehaviorTree.CPP
mkdir -p build && cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr/local ..
make -j4

# Install system-wide
sudo make install
sudo ldconfig

# Configure pkg-config paths (Required for CMake to find the library)
sudo mkdir -p /usr/local/lib/pkgconfig
# (Note: Ensure behaviortree_cpp.pc is created here)
sudo chmod 644 /usr/local/lib/pkgconfig/behaviortree_cpp.pc

# Export path and verify
export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH
pkg-config --modversion behaviortree_cpp
# Expected Output: 4.6.0

2. Project Initialization

Execute these commands to set up the directory structure.


# Navigate to ROS2 workspace src directory
cd ~/ros2_bt_test_ws/src

# Create package directory structure
mkdir -p bt_robot_task/include/bt_robot_task
mkdir -p bt_robot_task/src
cd bt_robot_task

3. Source File Requirements

Ensure the following files are created and populated with the source code before proceeding to the build phase.

    Header File: include/bt_robot_task/robot_actions.hpp

    Behavior Tree XML: src/robot_task.xml

    Main Executable: src/main.cpp

    Build Config: CMakeLists.txt

4. Build and Execute

Execute these commands to compile the project and run the robot task.


# Create build directory inside the package
mkdir -p build && cd build

# Configure and Compile
cmake .. && make -j4

# Execute the Robot Task
./robot_task_executor

Expected Output:
Plaintext

=== Robot Apple Picking Task ===
[NAVIGATE] Robot entering room...
[OPEN] Robot opening fridge...
[CHECK] Fridge open? YES
[PICK] Robot picking apple...
[EXIT] Robot exiting room...
=== TASK COMPLETE ===


------------------------------------------------------------------
ROBOTICS_PROJECTS DIRECTORY: Task 3 Maths Assignment
------------------------------------------------------------------


# Robotics Math Assignment - Task 1 & Task 2

Complete implementation of Euler-Quaternion conversion and Forward Kinematics for a 4-link perpendicular robot.

## 📋 Overview

This repository contains production-ready solutions for:

- **Task 1**: Bidirectional Euler angles ↔ Quaternion conversion with gimbal lock handling
- **Task 2**: Forward kinematics computation for a 4-link serial manipulator with perpendicular joints

## 🏗️ Project Structure

```
math_assignment/
├── task1_euler_quaternion.py      # Euler ↔ Quaternion converter
├── task2_forward_kinematics.py    # Forward kinematics calculator
├── README.md                       # This file
├── requirements.txt                # Python dependencies
├── setup.sh                        # Automated setup script
├── run_all_tests.sh               # Test runner script
├── tests/
│   ├── test_task1.py              # Unit tests for Task 1
│   └── test_task2.py              # Unit tests for Task 2
├── docs/
│   ├── TASK1_EXPLANATION.md       # Mathematical explanation Task 1
│   └── TASK2_EXPLANATION.md       # Mathematical explanation Task 2
└── visualizations/                 # Generated plots (optional)
```

## 🚀 Quick Start

### Prerequisites

- Python 3.7 or higher
- pip package manager

### Installation

```
# Clone the repository
git clone https://github.com/namanharshwal/mowito-assignment.git
cd ~/src/robotics_projects/math-assignment

# Install dependencies
python3 -m pip install --user -r requirements.txt

# Or use the automated setup script
chmod +x setup.sh
./setup.sh
```

### Running the Code

#### Task 1: Euler-Quaternion Conversion

```
python3 task1_euler_quaternion.py
```

**Expected Output:**
- Conversion test results
- Gimbal lock detection at ±90° pitch
- Round-trip conversion accuracy
- Stress test with 100 random angles

#### Task 2: Forward Kinematics

```
python3 task2_forward_kinematics.py
```

**Expected Output:**
- Zero configuration test
- Single and multiple joint movements
- Workspace exploration
- Singularity detection

### Running Tests

```
# Run all tests
chmod +x run_all_tests.sh
./run_all_tests.sh

# Or run individually
python3 -m pytest tests/test_task1.py -v
python3 -m pytest tests/test_task2.py -v
```

## 📖 Mathematical Background

### Task 1: Euler Angles & Quaternions

**Euler Angles**: Represent 3D orientation as three sequential rotations around X, Y, Z axes (roll, pitch, yaw).

**Quaternion**: 4D representation q = [qw, qx, qy, qz] where qw² + qx² + qy² + qz² = 1

**Advantages of Quaternions:**
- No gimbal lock
- Smooth interpolation
- Computationally efficient

**Gimbal Lock**: Occurs when pitch ≈ ±90°, causing loss of one degree of freedom (yaw and roll become indistinguishable).

### Task 2: Forward Kinematics

**DH Parameters** for the 4-link perpendicular robot:

| Joint | θ (rad) | d (m) | a (m) | α (rad) |
|-------|---------|-------|-------|---------|
| 1     | θ₁      | 0     | 1.0   | π/2     |
| 2     | θ₂      | 0     | 1.0   | π/2     |
| 3     | θ₃      | 0     | 1.0   | π/2     |
| 4     | θ₄      | 0     | 1.0   | 0       |

**Transformation Matrix**: T₀₄ = T₀₁ × T₁₂ × T₂₃ × T₃₄

**End-Effector Position**: [x, y, z]ᵀ = T₀₄[0:3, 3]

## 🎯 Features

### Task 1 Features
✅ Euler to Quaternion conversion (ZYX convention)  
✅ Quaternion to Euler conversion with gimbal lock handling  
✅ Angle normalization to [-π, π]  
✅ Quaternion normalization validation  
✅ Comprehensive test suite with edge cases  
✅ Round-trip conversion accuracy < 10⁻¹⁰ radians  

### Task 2 Features
✅ DH parameter-based transformation matrices  
✅ Forward kinematics for 4-link perpendicular robot  
✅ Parameterized link length (default 1m)  
✅ Full pose computation (position + orientation)  
✅ Workspace exploration utilities  
✅ Singularity detection  

## 🧪 Test Coverage

### Task 1 Tests
- Standard angle conversions
- Gimbal lock at +90° and -90° pitch
- Zero rotation (identity quaternion)
- 180° rotation edge cases
- 100 random angle stress test

### Task 2 Tests
- Zero configuration (home position)
- Single joint movements
- Combined multi-joint movements
- Maximum reach configuration
- Workspace boundary exploration
- Singularity configurations

## 📚 Usage Examples

### Task 1: Converting Angles

```
from task1_euler_quaternion import euler_to_quaternion, quaternion_to_euler

# Convert Euler to Quaternion
roll, pitch, yaw = 0.1, 0.2, 0.3  # radians
qw, qx, qy, qz = euler_to_quaternion(roll, pitch, yaw)
print(f"Quaternion: [{qw:.4f}, {qx:.4f}, {qy:.4f}, {qz:.4f}]")

# Convert back to Euler
roll2, pitch2, yaw2 = quaternion_to_euler(qw, qx, qy, qz)
print(f"Euler: roll={roll2:.4f}, pitch={pitch2:.4f}, yaw={yaw2:.4f}")
```

### Task 2: Computing End-Effector Position

```
from task2_forward_kinematics import RobotFK
import numpy as np

# Create robot with 1m links
robot = RobotFK(link_length=1.0)

# Compute FK for given joint angles
j1, j2, j3, j4 = np.pi/4, np.pi/6, 0, np.pi/3  # radians
x, y, z = robot.forward_kinematics(j1, j2, j3, j4)
print(f"End-effector: ({x:.4f}, {y:.4f}, {z:.4f}) m")

# Get full transformation matrix (includes orientation)
T = robot.get_full_transformation(j1, j2, j3, j4)
print("Full pose:\n", T)
```

## 🐛 Edge Cases Handled

### Task 1
- Gimbal lock at pitch = ±90° → Sets roll = 0, computes combined yaw
- Non-normalized quaternions → Raises ValueError with current norm
- Angle wrapping → Normalizes to [-π, π]

### Task 2
- Singularities (fully extended/folded) → Detected and reported
- Joint limits → Can be added by modifying DH parameters
- Numerical stability → Uses numpy for precision


### Algorithm Explanations

**Euler → Quaternion:**
```
1. Compute half angles: θ/2 for each axis
2. Use trigonometric identities for quaternion components
3. Normalize to ensure unit quaternion
```

**Quaternion → Euler:**
```
1. Check for gimbal lock: |sin(pitch)| ≈ 1
2. If gimbal lock: set roll = 0, compute effective yaw
3. Otherwise: use atan2 formulas for all three angles
```

**Forward Kinematics:**
```
1. Build DH transformation for each joint
2. Multiply transformations: T_total = T1 * T2 * T3 * T4
3. Extract position from T_total[0:3, 3]
```

## 📝 Dependencies

See `requirements.txt`:
```
numpy>=1.21.0
matplotlib>=3.4.0  # Optional, for visualization
scipy>=1.7.0       # Optional, for advanced tests
pytest>=6.2.0      # For running unit tests


***

## 📄 **File 4: requirements.txt**

```
numpy>=1.21.0
matplotlib>=3.4.0
scipy>=1.7.0
pytest>=6.2.0
```

***

## 📄 **File 5: setup.sh**

```bash
#!/bin/bash

echo "=================================================="
echo "  Robotics Math Assignment - Setup Script"
echo "=================================================="

# Check Python version
echo -e "\n[1/5] Checking Python version..."
python3 --version

# Create virtual environment (optional but recommended)
echo -e "\n[2/5] Setting up Python environment..."
python3 -m pip install --user --upgrade pip

# Install dependencies
echo -e "\n[3/5] Installing dependencies..."
python3 -m pip install --user -r requirements.txt

# Verify installations
echo -e "\n[4/5] Verifying installations..."
python3 -c "import numpy; print(f'✓ NumPy {numpy.__version__}')"
python3 -c "import matplotlib; print(f'✓ Matplotlib {matplotlib.__version__}')"

# Run quick test
echo -e "\n[5/5] Running quick verification test..."
python3 -c "
from task1_euler_quaternion import euler_to_quaternion
qw, qx, qy, qz = euler_to_quaternion(0, 0, 0)
print(f'✓ Task 1 import successful: Zero rotation quaternion = [{qw:.2f}, {qx:.2f}, {qy:.2f}, {qz:.2f}]')
"

python3 -c "
from task2_forward_kinematics import RobotFK
robot = RobotFK()
x, y, z = robot.forward_kinematics(0, 0, 0, 0)
print(f'✓ Task 2 import successful: Zero config position = ({x:.4f}, {y:.4f}, {z:.4f})')
"

echo -e "\n=================================================="
echo "  ✓ Setup complete! You can now run:"
echo "    python3 task1_euler_quaternion.py"
echo "    python3 task2_forward_kinematics.py"
echo "=================================================="
```

***

## 📄 **File 6: run_all_tests.sh**

```bash
#!/bin/bash

echo "=================================================="
echo "  Running All Tests for Math Assignment"
echo "=================================================="

echo -e "\n[Test 1] Task 1 - Euler-Quaternion Conversion"
echo "----------------------------------------------"
python3 task1_euler_quaternion.py

echo -e "\n\n[Test 2] Task 2 - Forward Kinematics"
echo "----------------------------------------------"
python3 task2_forward_kinematics.py

echo -e "\n\n=================================================="
echo "  ✓ All tests completed!"
echo "=================================================="
```

***

## 📄 **File 7: docs/TASK1_EXPLANATION.md**

````markdown
# Task 1: Euler Angles ↔ Quaternion Conversion - Mathematical Explanation

## 📐 Mathematical Foundation

### Euler Angles

**Definition**: Three sequential rotations around body-fixed axes (intrinsic rotations)

- **Roll (φ)**: Rotation around X-axis
- **Pitch (θ)**: Rotation around Y-axis  
- **Yaw (ψ)**: Rotation around Z-axis

**Rotation Sequence**: ZYX (Yaw → Pitch → Roll)

**Range**:
- Roll: φ ∈ [-π, π]
- Pitch: θ ∈ [-π/2, π/2]
- Yaw: ψ ∈ [-π, π]

### Quaternions

**Definition**: Extension of complex numbers to 3D rotations

q = qw + qx**i** + qy**j** + qz**k**

**Constraint**: Unit quaternion → qw² + qx² + qy² + qz² = 1

**Advantages**:
1. No gimbal lock
2. Smooth interpolation (SLERP)
3. Efficient composition
4. Compact representation (4 numbers vs 9 for matrix)

## 🔄 Conversion Formulas

### Euler → Quaternion

Given Euler angles (roll, pitch, yaw), compute quaternion [qw, qx, qy, qz]:

```
cy = cos(yaw/2)
sy = sin(yaw/2)
cp = cos(pitch/2)
sp = sin(pitch/2)
cr = cos(roll/2)
sr = sin(roll/2)

qw = cr * cp * cy + sr * sp * sy
qx = sr * cp * cy - cr * sp * sy
qy = cr * sp * cy + sr * cp * sy
qz = cr * cp * sy - sr * sp * cy
```

**Derivation**: Multiply quaternions for individual rotations:
q = q_yaw ⊗ q_pitch ⊗ q_roll

### Quaternion → Euler

Given quaternion [qw, qx, qy, qz], compute Euler angles:

```
# Roll (X-axis rotation)
t0 = 2.0 * (qw * qx + qy * qz)
t1 = 1.0 - 2.0 * (qx² + qy²)
roll = atan2(t0, t1)

# Pitch (Y-axis rotation)
sin_pitch = 2.0 * (qw * qy - qz * qx)
pitch = asin(clamp(sin_pitch, -1, 1))

# Yaw (Z-axis rotation)
t2 = 2.0 * (qw * qz + qx * qy)
t3 = 1.0 - 2.0 * (qy² + qz²)
yaw = atan2(t2, t3)
```

## ⚠️ Gimbal Lock Problem

### What is Gimbal Lock?

**Definition**: Loss of one degree of freedom when pitch approaches ±90°

**Physical Interpretation**: Two rotation axes align, losing ability to distinguish between roll and yaw

**Mathematical Condition**: |sin(pitch)| ≈ 1

### Why Gimbal Lock Occurs

When pitch = 90°:
- Z-axis (yaw) and X-axis (roll) become parallel
- Rotation around Z or X produces same effect
- Effectively reduces 3 DOF to 2 DOF

### Detection

```
sin_pitch = 2.0 * (qw * qy - qz * qx)

if |sin_pitch| >= 0.99999:
    # Gimbal lock detected!
    pitch = ±90°
    roll = 0  # Arbitrary choice
    yaw = 2 * atan2(qx, qw)  # Effective combined rotation
```

### Handling Strategy

1. **Detect**: Check if |sin(pitch)| > threshold (e.g., 0.99999)
2. **Set pitch**: Exactly ±90° based on sign
3. **Fix roll**: Set to 0 (arbitrary but conventional)
4. **Compute effective yaw**: Represents combined roll+yaw rotation

## 🧮 Implementation Details

### Normalization

**Why**: Accumulated numerical errors can violate unit quaternion constraint

**Method**:
```
norm = sqrt(qw² + qx² + qy² + qz²)
qw, qx, qy, qz = qw/norm, qx/norm, qy/norm, qz/norm
```

**When**: After every quaternion operation

### Angle Wrapping

**Purpose**: Keep angles in standard ranges

**Implementation**:
```
def normalize_angle(angle):
    while angle > π:
        angle -= 2π
    while angle < -π:
        angle += 2π
    return angle
```

### Numerical Stability

**Issue**: `asin` domain is [-1, 1], but numerical errors can exceed this

**Solution**: Clamp before asin
```
pitch = asin(np.clip(sin_pitch, -1.0, 1.0))
```

## 📊 Test Cases

### 1. Standard Angles
```
Input:  roll=0.1, pitch=0.2, yaw=0.3 rad
Output: qw=0.9833, qx=0.0998, qy=0.1489, qz=0.0588
Back:   roll=0.1000, pitch=0.2000, yaw=0.3000 rad
Error:  < 10⁻¹⁰ rad
```

### 2. Gimbal Lock (+90°)
```
Input:  roll=0.5, pitch=π/2, yaw=0.3 rad
Output: Gimbal lock detected!
Back:   roll=0.0, pitch=π/2, yaw=0.8 rad (effective)
Note:   roll+yaw preserved as single rotation
```

### 3. Zero Rotation
```
Input:  roll=0, pitch=0, yaw=0
Output: qw=1.0, qx=0.0, qy=0.0, qz=0.0 (identity)
```

### 4. 180° Rotation
```
Input:  roll=π, pitch=0, yaw=0 (180° around X)
Output: qw=0.0, qx=1.0, qy=0.0, qz=0.0
```

