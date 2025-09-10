import os

import ament_index_python

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import Node

# Path to the launch files and directories that we will use
_MICROSTRAIN_LAUNCH_FILE = os.path.join(ament_index_python.packages.get_package_share_directory('microstrain_inertial_driver'), 'launch', 'microstrain_launch.py')
_CV7_INS_PARAMS_FILE = os.path.join(ament_index_python.packages.get_package_share_directory('modular_ins'), 'config', 'cv7_ins', 'cv7_ins_mosaic_h.yml')
_RVIZ_DISPLAY_FILE = os.path.join(ament_index_python.packages.get_package_share_directory('modular_ins'), 'config', 'cv7_ins', 'display.rviz')

_CONFIG_DIRECTORY = os.path.join(ament_index_python.packages.get_package_share_directory('modular_ins'), 'config')
_SEPT_CONFIG_FILE = os.path.join(_CONFIG_DIRECTORY, 'mosaic_h.yaml') #TODO check if needed

def generate_launch_description():
  return LaunchDescription([
    # Microstrain node
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(_MICROSTRAIN_LAUNCH_FILE),
      launch_arguments={
        'configure': 'true',
        'activate': 'true',
        'params_file': _CV7_INS_PARAMS_FILE,
        'namespace': '/cv7_ins', #need to verify
	      'debug': 'true',
      }.items()
    ),
    
    # Septentrio Mosaic H node
    Node(
      package='septentrio_gnss_driver',
      executable='septentrio_gnss_driver_node',
      output='both',
      parameters=[_SEPT_CONFIG_FILE],
      remappings=[
        ('/navsatfix','/cv7_ins/ext/llh_position'),
        ('/twist','/cv7_ins/ext/velocity_enu')
      ]),

    # Publish a static transform for where the our gps is mounted on base_link.
    # You should replace this with actual transforms for where your aiding sensors are
    Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      output='screen',
      arguments=[
          "--x", "0.15",
          "--y", "-0.45",
          "--z", "1.58",
          "--roll", "0",
          "--pitch", "0",
          "--yaw", "0",
          "--frame-id", "base_link",
          "--child-frame-id", "gps"
        ]
    ),

    # Publish a static transform for where the CV7-INS is mounted on base_link.
    # Unless the CV7-INS is mounted exactly at base_link, you should change this to be accurate to your setup
    Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      output='screen',
      arguments=[
          "--x", "-0.37",
          "--y", "-0.55",
          "--z", "0.63",
          "--roll", "0",
          "--pitch", "0",
          "--yaw", "0",
          "--frame-id", "base_link",
          "--child-frame-id", "cv7_ins_link"
        ]
    ),

    # Run rviz to view the state of the application
    Node(
      package='rviz2',
      executable='rviz2',
      output='screen',
      arguments=[
        '-d', _RVIZ_DISPLAY_FILE
      ]
    ),
  ])
