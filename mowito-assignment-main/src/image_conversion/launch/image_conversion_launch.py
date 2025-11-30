#!/usr/bin/env python3
"""
Launch file for Image Conversion Node with usb_cam.
Launches usb_cam + image_conversion + image_saver.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for image conversion system."""
    
    # Declare launch arguments
    input_topic_arg = DeclareLaunchArgument(
        'input_image_topic',
        default_value='/image_raw',
        description='Topic name for input camera images'
    )
    
    output_topic_arg = DeclareLaunchArgument(
        'output_image_topic',
        default_value='/image_converted',
        description='Topic name for output converted images'
    )
    
    mode_arg = DeclareLaunchArgument(
        'initial_mode',
        default_value='true',
        description='Initial conversion mode (true=grayscale, false=color)'
    )
    
    # Get launch configurations
    input_topic = LaunchConfiguration('input_image_topic')
    output_topic = LaunchConfiguration('output_image_topic')
    initial_mode = LaunchConfiguration('initial_mode')
    
    # USB Cam Node
    usb_cam_node = Node(
        package='dummy_usb_cam',
        executable='dummy_camera_publisher',
        name='usb_cam',
        output='screen',
        parameters=[
            {'camera_name': 'default_cam'},
            {'camera_info_url': 'package://usb_cam/config/camera_info.yaml'},
            {'framerate': 30.0},
            {'frame_id': 'camera_frame'},
        ]
    )
    
    # Image Conversion Node
    image_conversion_node = Node(
        package='image_conversion',
        executable='image_conversion_node',
        name='image_conversion',
        output='screen',
        parameters=[
            {'input_topic': input_topic},
            {'output_topic': output_topic},
            {'grayscale_mode': initial_mode},
        ],
        remappings=[
            ('/image_raw', input_topic)
        ]
    )

    # Image Saver Node (ADDED)
    image_saver_node = Node(
        package='image_conversion',
        executable='image_saver',
        name='image_saver',
        output='screen'
    )
    
    return LaunchDescription([
        input_topic_arg,
        output_topic_arg,
        mode_arg,
        usb_cam_node,
        image_conversion_node,
        image_saver_node,    # <-- ADDED HERE
    ])

