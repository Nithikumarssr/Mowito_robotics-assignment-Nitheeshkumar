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
