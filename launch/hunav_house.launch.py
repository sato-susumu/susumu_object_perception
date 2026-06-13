# Phase A: HuNavSim-controlled pedestrians in the multi-room house world.
#
# This launch reuses the HuNav Gazebo wrapper world-generation pipeline:
#   1. hunav_loader            -> loads the 5-agent config (agents_house.yaml)
#   2. hunav_gazebo_world_generator -> merges house.world + agents into
#                                       generatedWorld.world (with the HuNav plugin)
#   3. gzserver/gzclient       -> runs the generated world
#   4. hunav_agent_manager     -> drives agent behaviors (Social Force Model)
#
# No robot is spawned in this phase. Robot + Nav2 are added in later phases.

from os import path, pathsep

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, Shutdown,
                            RegisterEventHandler, TimerAction, LogInfo,
                            SetEnvironmentVariable)
from launch.substitutions import (PathJoinSubstitution, LaunchConfiguration,
                                   PythonExpression, EnvironmentVariable)
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------
    declare_agents_conf = DeclareLaunchArgument(
        'configuration_file', default_value='agents_house.yaml',
        description='Agent (pedestrian) configuration file in susumu_sim/config')
    declare_base_world = DeclareLaunchArgument(
        'base_world', default_value='house.world',
        description='Base world file (taken from hunav_gazebo_wrapper/worlds)')
    declare_gz_obs = DeclareLaunchArgument(
        'use_gazebo_obs', default_value='true',
        description='Let agents treat the closest Gazebo model as an obstacle')
    declare_rate = DeclareLaunchArgument(
        'update_rate', default_value='100.0',
        description='HuNav plugin update rate (Hz)')
    declare_robot_name = DeclareLaunchArgument(
        'robot_name', default_value='turtlebot3',
        description='Name of the robot Gazebo model the HuNav plugin tracks')
    declare_global_frame = DeclareLaunchArgument(
        'global_frame_to_publish', default_value='map',
        description='Global frame in which agent positions are published')
    declare_use_navgoal = DeclareLaunchArgument(
        'use_navgoal_to_start', default_value='False',
        description='Start agents only after a navigation goal is received')
    declare_navgoal_topic = DeclareLaunchArgument(
        'navgoal_topic', default_value='goal_pose',
        description='Topic carrying the robot navigation goal')
    declare_ignore_models = DeclareLaunchArgument(
        'ignore_models', default_value='ground_plane',
        description='Gazebo models the agents should ignore as obstacles')
    declare_verbose = DeclareLaunchArgument(
        'verbose', default_value='False',
        description='Increase Gazebo terminal output')
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='False',
        description='Open RViz to visualize the published people markers')
    # Phase A publishes a static map->odom TF so frames resolve without Nav2.
    declare_navigation = DeclareLaunchArgument(
        'navigation', default_value='False',
        description='Set True when an external localization/navigation provides map->odom')

    agents_conf = LaunchConfiguration('configuration_file')
    base_world = LaunchConfiguration('base_world')
    gz_obs = LaunchConfiguration('use_gazebo_obs')
    rate = LaunchConfiguration('update_rate')
    robot_name = LaunchConfiguration('robot_name')
    global_frame = LaunchConfiguration('global_frame_to_publish')
    use_navgoal = LaunchConfiguration('use_navgoal_to_start')
    navgoal_topic = LaunchConfiguration('navgoal_topic')
    ignore_models = LaunchConfiguration('ignore_models')
    navigation = LaunchConfiguration('navigation')

    # ------------------------------------------------------------------
    # 1) hunav_loader: read the agent configuration
    # ------------------------------------------------------------------
    agent_conf_file = PathJoinSubstitution([
        FindPackageShare('susumu_sim'), 'config', agents_conf])

    hunav_loader_node = Node(
        package='hunav_agent_manager',
        executable='hunav_loader',
        output='screen',
        parameters=[agent_conf_file])

    # ------------------------------------------------------------------
    # 2) world generator: house.world + agents -> generatedWorld.world
    #    (the generator looks for base_world inside hunav_gazebo_wrapper/worlds)
    # ------------------------------------------------------------------
    world_file = PathJoinSubstitution([
        FindPackageShare('hunav_gazebo_wrapper'), 'worlds', base_world])

    hunav_worldgen_node = Node(
        package='hunav_gazebo_wrapper',
        executable='hunav_gazebo_world_generator',
        output='screen',
        parameters=[{'base_world': world_file},
                    {'use_gazebo_obs': gz_obs},
                    {'update_rate': rate},
                    {'robot_name': robot_name},
                    {'global_frame_to_publish': global_frame},
                    {'use_navgoal_to_start': use_navgoal},
                    {'navgoal_topic': navgoal_topic},
                    {'ignore_models': ignore_models}])

    worldgen_after_loader = RegisterEventHandler(
        OnProcessStart(
            target_action=hunav_loader_node,
            on_start=[
                LogInfo(msg='hunav_loader started; launching world generator in 2 s...'),
                TimerAction(period=2.0, actions=[hunav_worldgen_node])]))

    # ------------------------------------------------------------------
    # 3) Gazebo with the generated world
    # ------------------------------------------------------------------
    # The hunav_gazebo_wrapper env-hooks already prepend its models/worlds to the
    # GAZEBO_* paths when the workspace is sourced. We additionally make sure the
    # wrapper media (human meshes) and the standard gazebo plugin dir are present.
    wrapper_models = PathJoinSubstitution([
        FindPackageShare('hunav_gazebo_wrapper'), 'models'])
    wrapper_media = PathJoinSubstitution([
        FindPackageShare('hunav_gazebo_wrapper'), 'media', 'models'])

    set_env_model = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[EnvironmentVariable('GAZEBO_MODEL_PATH', default_value=''),
               pathsep, wrapper_models, pathsep, wrapper_media])
    set_env_resource = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value=[EnvironmentVariable('GAZEBO_RESOURCE_PATH', default_value=''),
               pathsep, wrapper_models, pathsep, wrapper_media])

    config_file = path.join(
        get_package_share_directory('hunav_gazebo_wrapper'), 'launch', 'params.yaml')

    generated_world = PathJoinSubstitution([
        FindPackageShare('hunav_gazebo_wrapper'), 'worlds', 'generatedWorld.world'])

    gzserver_cmd = [
        'gzserver ', generated_world,
        _boolean_command('verbose'), '',
        '-s ', 'libgazebo_ros_init.so',
        '-s ', 'libgazebo_ros_factory.so',
        '--ros-args', '--params-file', config_file]

    gzclient_cmd = ['gzclient', _boolean_command('verbose'), ' ']

    gzserver_process = ExecuteProcess(
        cmd=gzserver_cmd, output='screen', shell=True, on_exit=Shutdown())
    gzclient_process = ExecuteProcess(
        cmd=gzclient_cmd, output='screen', shell=True, on_exit=Shutdown())

    gazebo_after_worldgen = RegisterEventHandler(
        OnProcessStart(
            target_action=hunav_worldgen_node,
            on_start=[
                LogInfo(msg='world generated; launching Gazebo in 2 s...'),
                TimerAction(period=2.0, actions=[gzserver_process, gzclient_process])]))

    # ------------------------------------------------------------------
    # 4) HuNav behavior manager (Social Force Model driver)
    # ------------------------------------------------------------------
    hunav_manager_node = Node(
        package='hunav_agent_manager',
        executable='hunav_agent_manager',
        name='hunav_agent_manager',
        output='screen',
        parameters=[{'use_sim_time': True}])

    # When no navigation is running, publish a static map->odom so TF is complete.
    static_tf_node = Node(
        package='tf2_ros', executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        condition=UnlessCondition(navigation))

    ld = LaunchDescription()
    for a in (declare_agents_conf, declare_base_world, declare_gz_obs, declare_rate,
              declare_robot_name, declare_global_frame, declare_use_navgoal,
              declare_navgoal_topic, declare_ignore_models, declare_verbose,
              declare_use_rviz, declare_navigation):
        ld.add_action(a)

    ld.add_action(set_env_model)
    ld.add_action(set_env_resource)

    ld.add_action(hunav_loader_node)
    ld.add_action(worldgen_after_loader)
    ld.add_action(hunav_manager_node)
    ld.add_action(gazebo_after_worldgen)
    ld.add_action(static_tf_node)
    return ld


def _boolean_command(arg):
    return PythonExpression(
        ['"--', arg, '" if "true" == "', LaunchConfiguration(arg), '" else ""'])
