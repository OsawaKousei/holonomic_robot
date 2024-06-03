import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_name = LaunchConfiguration('world_name', default='test_world')
    pkg_share_dir = get_package_share_directory('holonomic_sim')
    model_path = os.path.join(pkg_share_dir, "models")

    #ignition gazeboがモデルにアクセスできるように設定
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',value=[
        os.path.join("/opt/ros/humble", "share"),
        ":" +
        model_path])

    #ロボットをスポーンさせる設定
    ignition_spawn_entity = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        arguments=['-entity', 'HolonomicRobo',
                   '-name', 'HolonomicRobo',
                   #ロボットのsdfファイルを指定
                   '-file', PathJoinSubstitution([
                        pkg_share_dir,
                        "models", "HolonomicRobo", "model.sdf"]),
                    #ロボットの位置を指定
                   '-allow_renaming', 'true',
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '1.0',
                   ],
        )
    
    #ワールドのsdfファイルを設定(worldタグのあるsdfファイル)
    world = os.path.join(pkg_share_dir,"models","worlds", "holonomic_test.sdf")

    #ignition gazeboの起動設定
    ign_gz = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_ign_gazebo'),
                              'launch', 'ign_gazebo.launch.py')]),
            launch_arguments=[('ign_args', [' -r -v 3 ' +
                              world
                             ])])
    
    #ros_ign_bridgeの起動設定
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        parameters=[{
            #brigdeの設定ファイルを指定
            'config_file': os.path.join(pkg_share_dir, 'config', 'teleop.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
            'qos_overrides./odom.publisher.durability': 'transient_local',
        },{'use_sim_time': use_sim_time}],
        remappings=[
            ("/odom/tf", "tf"),
        ],
        output='screen'
    )
    
    #ロボットのsdfファイルのパスを取得
    sdf = os.path.join(model_path, 'HolonomicRobo', 'model.sdf')

    #xacroでsdfファイルをurdfに変換
    doc = xacro.parse(open(sdf))
    xacro.process_doc(doc)

    #robot_state_publsherの起動設定
    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='both',
            parameters=[{'use_sim_time': use_sim_time,
                         'robot_description': doc.toxml()}]) # type: ignore

    #rviz2の設定フィルのパスを取得
    rviz_config_dir = os.path.join(
        pkg_share_dir,
        'config',
        'holonomic_test.rviz')
    
    #rviz2の起動設定
    rviz2 = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen')
    
    #rqt
    rqt = Node(
            package='rqt_publisher',
            executable='rqt_publisher',
            name='rqt_publisher',
            output='screen')
    
    return LaunchDescription([
        ign_resource_path,
        ignition_spawn_entity,
        ign_gz,
                             
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),

        bridge,

        DeclareLaunchArgument(
            'world_name',
            default_value=world_name,
            description='World name'),

        rqt,

        # robot_state_publisher,
        # rviz2,
    ])