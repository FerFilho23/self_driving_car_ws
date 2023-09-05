import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():

    package_dir = get_package_share_directory('prius_sdc_pkg')
    world_file = os.path.join(package_dir, 'worlds', 'car.world')

    return LaunchDescription([
        
        ExecuteProcess(
            cmd = ['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
            output = 'screen',
        ),

        # Node(
        #     package = '',
        #     executable = '',
        #     name = '',
        #     output = 'screen',
        # )
    ])