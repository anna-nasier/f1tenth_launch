from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os
import launch
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    pub = Node(name = "obstacle_detection", package = "f1tenth_launch", executable = "obstacle_detection.py")
    
    
    return launch.LaunchDescription([
        DeclareLaunchArgument("use_multithread", default_value="false"), # changed from true 
        pub
        ])