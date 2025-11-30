This repository contains my complete implementation of all robotics assignments provided by Mowito.
Task 1 implements a robust Euler-to-Quaternion and Quaternion-to-Euler conversion module in Python, including gimbal-lock handling, angle normalization, and a comprehensive automated test suite.
Task 2 implements forward kinematics for a 4-link perpendicular-joint manipulator using Denavit–Hartenberg parameters, computing homogeneous transformations and validating workspace coverage through structured test cases.
A dedicated ros_comms C++ package demonstrates ROS2 communication primitives, including a publisher node, a subscriber node, and a custom AddTwoInts service with both server and client implementations.
Task 3 implements the Image Conversion Node in ROS2, subscribing to /image_raw, converting between grayscale and color based on a SetBool service, and publishing processed frames to /image_converted.
A dummy USB camera node is integrated to simulate a live video source, allowing the full perception pipeline to be tested without physical hardware.
An image saver node and an auto image saver node were developed to store grayscale and color outputs to disk, enabling quick visual verification and dataset-style logging of processed images.
The launch configuration was extended to bring up the camera, image conversion node, and saver node together as a single pipeline, providing a clean, reproducible test setup.
During development, I resolved build issues, corrected package structure, fixed CMake/ament dependencies, and ensured that all ROS2 packages build cleanly with colcon in a Humble workspace.
Overall, this project demonstrates practical skills in robotics math, ROS2 pub–sub and services, image processing pipelines, and full-stack debugging from Python scripts to C++ nodes and launch files.
Thank you
