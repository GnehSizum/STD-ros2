# <launch>
# <!-- Launch file for Livox AVIA LiDAR -->

#   <arg name="rviz" default="true" />
#   <rosparam command="load" file="$(find std_detector)/config/config_online.yaml" />
#   <group if="$(arg rviz)">
#     <node launch-prefix="nice" pkg="rviz" type="rviz" name="rviz" args="-d $(find std_detector)/rviz_cfg/online.rviz" />
#   </group>

#   <node
# 		pkg="std_detector"
# 		type="online_demo"
# 		name="online_demo"
# 		output="screen"
# 	/>

# </launch>

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    package_path = get_package_share_directory('std_detector')
    config_path = os.path.join(package_path, 'config', 'config_online.yaml')

    decalre_config_path_cmd = DeclareLaunchArgument(
        'config_file', default_value=config_path,
        description='Config file'
        )

    std_node_cmd = Node(
        package='std_detector',
        executable='std_detector_node',
        # prefix=['gnome-terminal -- gdb -ex run --args'],
        output='screen'
        )

    ld = LaunchDescription()

    ld.add_action(decalre_config_path_cmd)
    ld.add_action(std_node_cmd)

    return ld
