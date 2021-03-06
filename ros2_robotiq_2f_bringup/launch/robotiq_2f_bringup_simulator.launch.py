import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, actions
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
     urdf1 = os.path.join(get_package_share_directory('ros2_robotiq_2f_description'),
                        'urdf', 'robotiq_2f_85.urdf')
     rviz_config_file = os.path.join(get_package_share_directory('ros2_robotiq_2f_bringup'),
                        'config', 'robotiq_2f.rviz')

     return LaunchDescription([

           DeclareLaunchArgument(
               name="gui",
               default_value="True",
               description="Launch GUI?"),

          DeclareLaunchArgument(
               name="rviz",
               default_value="True",
               description="Launch RViz?"),

          Node(package='ros2_robotiq_2f_state_publisher', node_executable='ros2_robotiq_2f_state_publisher',
               output='screen', arguments=[urdf1]),
          Node(package='ros2_robotiq_2f_simulator', node_executable='ros2_robotiq_2f_simulator',
               output='screen'), 
          Node(package='ros2_robotiq_2f_gui', node_executable='ros2_robotiq_2f_gui',
               output='screen', condition = IfCondition(LaunchConfiguration("gui"))),
          Node(package='ros2_robotiq_2f_utilities', node_executable='ros2_robotiq_2f_utilities',
               output='screen'),
          Node(package='rviz2', node_executable='rviz2', arguments=['-d', rviz_config_file],
               output='screen', condition = IfCondition(LaunchConfiguration("rviz")))
     ])
