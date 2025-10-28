from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    yt_recorder_share = FindPackageShare('yt_recorder')

    default_config_path = PathJoinSubstitution([yt_recorder_share, 'config', 'params.yaml'])
    default_mapviz_config_path = PathJoinSubstitution([yt_recorder_share, 'mapviz', 'yt-recorder.mvc'])

    config_arg = DeclareLaunchArgument(
        name='config',
        default_value=default_config_path,
        description="Path to the configuration file for the stonefish simulator"
    )

    mapviz_config_arg = DeclareLaunchArgument(
        name='mapviz_config',
        default_value=default_mapviz_config_path,
        description='Path to the MapViz configuration file'
    )

    mapviz_node = Node(
        package="mapviz",
        executable="mapviz",
        name="mapviz",
        arguments=["-d", LaunchConfiguration('mapviz_config')]
    )

    init_origin_node = Node(
        package="swri_transform_util",
        executable="initialize_origin.py",
        name="initialize_origin",
        parameters=[LaunchConfiguration('config')]
    )

    tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="swri_transform",
        arguments=["0", "0", "0", "0", "0", "0", "map", "origin"]
    )

    sail_1 = Node(
        package='yt_recorder',
        executable='yt_recorder',
        name='sail_1',
        parameters=[LaunchConfiguration('config')]
    )

    sail_2 = Node(
        package='yt_recorder',
        executable='yt_recorder',
        name='sail_2',
        parameters=[LaunchConfiguration('config')]
    )

    return LaunchDescription([
        config_arg,
        mapviz_config_arg,
        mapviz_node,
        init_origin_node,
        tf_node,
        sail_1,
        sail_2
    ])
