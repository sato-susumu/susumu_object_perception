# Person-following pipeline (LiDAR-only; does NOT use HuNavSim ground truth).
#
#   /velodyne_points (3D LiDAR)
#     -> person_detector_node   cluster moving objects -> /perception/persons
#     -> follow_person_node      lock 1 target, send Nav2 goals to walk on its RIGHT
#
# Meant to run alongside the simulation (Gazebo + people + Nav2). Use
# simulation.launch.py with follow:=true to launch everything together, or run
# this on top of an already-running simulation.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    fixed_frame = LaunchConfiguration('fixed_frame')
    input_topic = LaunchConfiguration('input_topic')
    side_offset = LaunchConfiguration('side_offset')

    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='True')
    declare_fixed_frame = DeclareLaunchArgument(
        'fixed_frame', default_value='odom',
        description='Frame in which detections/goals are expressed')
    declare_input_topic = DeclareLaunchArgument(
        'input_topic', default_value='/velodyne_points',
        description='3D LiDAR PointCloud2 topic')
    declare_side_offset = DeclareLaunchArgument(
        'side_offset', default_value='0.8',
        description='Lateral distance kept on the person\'s right [m]')

    detector = Node(
        package='susumu_sim',
        executable='person_detector_node.py',
        name='person_detector',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_topic': input_topic,
            'fixed_frame': fixed_frame,
        }])

    follower = Node(
        package='susumu_sim',
        executable='follow_person_node.py',
        name='follow_person',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'fixed_frame': fixed_frame,
            'side_offset': side_offset,
        }])

    return LaunchDescription([
        declare_use_sim_time, declare_fixed_frame,
        declare_input_topic, declare_side_offset,
        detector, follower,
    ])
