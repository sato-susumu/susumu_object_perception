# 複数の部屋がつながった house world で HuNavSim が制御する歩行者を動かす。
#
# この launch は HuNav Gazebo wrapper のワールド生成パイプラインを再利用する:
#   1. hunav_loader            -> 5人のエージェント設定（agents_house.yaml）を読み込む
#   2. hunav_gazebo_world_generator -> house.world + エージェントを統合し
#                                       generatedWorld.world を生成（HuNav プラグイン付き）
#   3. gzserver/gzclient       -> 生成したワールドを実行する
#   4. hunav_agent_manager     -> エージェントの behavior を駆動（Social Force Model）
#
# この launch ではロボットは spawn しない。ロボット + Nav2 は別の launch で追加する。

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
    # Launch 引数
    # ------------------------------------------------------------------
    declare_agents_conf = DeclareLaunchArgument(
        'configuration_file', default_value='agents_cafe.yaml',
        description='susumu_object_perception/config 内のエージェント（歩行者）設定ファイル')
    declare_base_world = DeclareLaunchArgument(
        'base_world', default_value='cafe.world',
        description='ベースとなる world ファイル（hunav_gazebo_wrapper/worlds から取得）')
    declare_gz_obs = DeclareLaunchArgument(
        'use_gazebo_obs', default_value='true',
        description='最寄りの Gazebo モデルを障害物としてエージェントに扱わせる')
    declare_rate = DeclareLaunchArgument(
        'update_rate', default_value='100.0',
        description='HuNav プラグインの更新レート（Hz）')
    declare_robot_name = DeclareLaunchArgument(
        'robot_name', default_value='turtlebot3',
        description='HuNav プラグインが追跡するロボットの Gazebo モデル名')
    declare_global_frame = DeclareLaunchArgument(
        'global_frame_to_publish', default_value='map',
        description='エージェント位置を publish するグローバルフレーム')
    declare_use_navgoal = DeclareLaunchArgument(
        'use_navgoal_to_start', default_value='False',
        description='ナビゲーションゴールを受け取ってからエージェントを動かし始める')
    declare_navgoal_topic = DeclareLaunchArgument(
        'navgoal_topic', default_value='goal_pose',
        description='ロボットのナビゲーションゴールを運ぶトピック')
    declare_ignore_models = DeclareLaunchArgument(
        'ignore_models', default_value='ground_plane',
        description='エージェントが障害物として無視すべき Gazebo モデル')
    declare_verbose = DeclareLaunchArgument(
        'verbose', default_value='False',
        description='Gazebo のターミナル出力を増やす')
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='False',
        description='publish された people マーカーを可視化する RViz を開く')
    # 外部のナビゲーションがない場合、静的な map->odom TF を publish して
    # Nav2 なしでもフレームが解決できるようにする。
    declare_navigation = DeclareLaunchArgument(
        'navigation', default_value='False',
        description='外部の自己位置推定/ナビゲーションが map->odom を提供する場合は True にする')

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
    # 1) hunav_loader: エージェント設定を読み込む
    # ------------------------------------------------------------------
    agent_conf_file = PathJoinSubstitution([
        FindPackageShare('susumu_object_perception'), 'config', agents_conf])

    hunav_loader_node = Node(
        package='hunav_agent_manager',
        executable='hunav_loader',
        output='screen',
        parameters=[agent_conf_file])

    # ------------------------------------------------------------------
    # 2) ワールド生成: house.world + エージェント -> generatedWorld.world
    #    （生成器は hunav_gazebo_wrapper/worlds 内の base_world を探す）
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
    # 3) 生成したワールドで Gazebo を起動
    # ------------------------------------------------------------------
    # hunav_gazebo_wrapper の env-hook は、ワークスペースを source した時点で
    # models/worlds を GAZEBO_* パスの先頭に追加済み。ここではさらに wrapper の
    # メディア（人物メッシュ）と標準の gazebo プラグインディレクトリが含まれることを確認する。
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
    # 4) HuNav behavior マネージャ（Social Force Model の駆動）
    # ------------------------------------------------------------------
    hunav_manager_node = Node(
        package='hunav_agent_manager',
        executable='hunav_agent_manager',
        name='hunav_agent_manager',
        output='screen',
        parameters=[{'use_sim_time': True}])

    # ナビゲーションが動いていないときは静的な map->odom を publish して TF を完結させる。
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
