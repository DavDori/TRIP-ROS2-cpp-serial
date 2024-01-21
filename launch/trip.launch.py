import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   config = os.path.join(
      get_package_share_directory('trip_interface'),
      'config',
      'robot_model.yaml',
      'communication.yaml',
      )

   return LaunchDescription([
      Node(
        package='trip_interface',
        executable='trip_unicycle',
        name='trip_unicycle',
        output="screen",
        parameters=[config]
      )
   ])
