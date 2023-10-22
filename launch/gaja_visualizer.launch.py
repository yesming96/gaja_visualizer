#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
  ld = LaunchDescription();

  talk = Node(
    package='gaja_visualizer',
    executable='talker',
    name='talker')

  listen = Node(
    package='gaja_visualizer',
    executable='listener',
    name='listener')
  
  # control = Node(
  #   package='gaja_visualizer',
  #   executable='controller',
  #   name='controller')
  ld.add_action(talk)
  ld.add_action(listen)
  # ld.add_action(control)

  return ld;