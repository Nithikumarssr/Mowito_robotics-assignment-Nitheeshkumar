VQ-BeT Task 2: ROS2 Image Conversion Package

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
