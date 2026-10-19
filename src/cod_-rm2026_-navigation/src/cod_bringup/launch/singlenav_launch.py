import os
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            GroupAction, IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 获取包的共享目录
    bring_up_dir = get_package_share_directory('cod_bringup')

    # 源目录(realpath 解析软链接): 建图后新保存的地图直接落在 src 的 maps 里,
    # install 里只有构建时的快照, 新文件不会自动同步过去
    src_bringup_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    # 工作区根目录: auto_level_map.py 放在那里
    ws_root = os.path.abspath(os.path.join(src_bringup_dir, '..', '..', '..', '..'))
    # 默认加载"最近保存的地图"(maps/latest.yaml): 建完图执行 ./save_map.sh 更新它,
    # 之后直接 ros2 launch 就用新图; 没有 latest 时退回 rmul2026.yaml
    default_map = os.path.join(src_bringup_dir, 'maps', 'latest.yaml')
    if not os.path.isfile(default_map):
        default_map = os.path.join(src_bringup_dir, 'maps', 'rmul2026.yaml')

    rviz_config_file = os.path.join(bring_up_dir, 'rviz', 'cod_nav.rviz')

    fast_lio_config_file = 'mid360.yaml'
    # 声明启动参数
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true')
    declare_slam_params_file = DeclareLaunchArgument(
        'slam_params_file', default_value=os.path.join(bring_up_dir,'params','mapper_params_async.yaml')
    )
    declare_nav2_params_file = DeclareLaunchArgument(
        'nav2_params_file',default_value=os.path.join(bring_up_dir,'params','singlenav2_params.yaml')
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='是否启动RViz; 无显示器时用 rviz:=false')
    declare_map_file = DeclareLaunchArgument(
        'map', default_value=default_map,
        description='要加载的地图yaml路径(默认自动用 maps/latest.yaml); 临时换别的图: map:=<绝对路径>/xxx.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('rviz')
    slam_params_file = LaunchConfiguration('slam_params_file')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    map_yaml_file = LaunchConfiguration('map')

    # 定义节点和包含的launch文件
    load_nodes = GroupAction(
        actions=[
            Node(
                package='cpp_lidar_filter',
                executable='lidar_filter_node',
                name='my_lidar_filter',
                output='screen',
                parameters=[{
                    'input_topic': '/livox/lidar',
                    'output_topic': '/livox/lidar_filtered',
                    'min_x': -0.2, 'max_x': 0.2,
                    'min_y': -0.2, 'max_y': 0.4,
                    'min_z': -0.1, 'max_z': 0.2,
                    'negative': True,   # 挖掉车身
                    'leaf_size': 0.05   # 降采样
                }]
            ),
            Node(
                    package="small_point_lio",
                    executable="small_point_lio_node",
                    name="small_point_lio",
                    output="screen",
                    parameters=[
                        PathJoinSubstitution(
                            [
                                FindPackageShare("small_point_lio"),
                                "config",
                                "mid360.yaml",
                            ]
                        )
                    ],
            ),

            # 自动水平补偿: 用地平面实测 map->odom 并持续发布 (yaw_offset=-0.5 保持原图对齐)
            ExecuteProcess(
                cmd=["python3", os.path.join(ws_root, "auto_level_map.py"),
                     "--ros-args", "-p", "yaw_offset:=-0.5"],
                output="screen",
            ),
            #动态大yaw: 订阅电控 /yaw, 发布 base_link→livox_frame 装位TF(含22°倾斜补偿)
            Node(
                package="small_point_lio",
                executable="yaw_tf_broadcaster",
                name="yaw_tf_broadcaster",
                output="screen",
                parameters=[{
                    "mount_x": 0.05289,
                    "mount_y": 0.15191,
                    "mount_z": 0.15691,
                    "mount_roll": 0.3839,   # 装位倾斜22°(绕x轴, 2026-09-16 实测)
                    "mount_pitch": 0.0,
                }],
            ),
            Node(
                package="fake_vel_transform",
                executable="fake_vel_transform_node",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time,"spin_speed": 1.0}],
            ),
            Node(
                package="cod_serial_ul26",
                executable="cod_serial",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(bring_up_dir,'launch','navigation_launch.py')),
                launch_arguments={
                                  'use_sim_time': "false",
                                  'autostart': "true",
                                  'params_file': nav2_params_file,
                                  'use_composition': 'False',
                                  'use_respawn': 'False',
                                  'container_name': 'nav2_container'}.items()
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(bring_up_dir,'launch','localization_launch.py')),
                launch_arguments={
                                  'use_sim_time': "false",
                                  'autostart': "true",
                                  'params_file': nav2_params_file,
                                  'map': map_yaml_file,
                                  'use_composition': 'False',
                                  'use_respawn': 'False',
                                  'container_name': 'nav2_container'}.items()
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d',rviz_config_file],
                output='screen',
                condition=IfCondition(use_rviz),
            ),
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_slam_params_file,
        declare_nav2_params_file,
        declare_map_file,
        declare_rviz,
        load_nodes
    ])