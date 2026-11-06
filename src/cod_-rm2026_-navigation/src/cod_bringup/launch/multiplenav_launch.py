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

    # 工作区根目录(realpath 解析软链接): auto_level_map.py 放在工作区根目录
    ws_root = os.path.abspath(os.path.join(
        os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
        '..', '..', '..', '..'))

    rviz_config_file = os.path.join(bring_up_dir, 'rviz', 'cod_nav.rviz')

    # 声明启动参数
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true')
    declare_slam_params_file = DeclareLaunchArgument(
        'slam_params_file', default_value=os.path.join(bring_up_dir,'params','mapper_params_online_async.yaml')
    )
    declare_nav2_params_file = DeclareLaunchArgument(
        'nav2_params_file',default_value=os.path.join(bring_up_dir,'params','multiplenav2_params.yaml')
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='是否启动RViz; 无显示器建图时用 rviz:=false')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('rviz')
    slam_params_file = LaunchConfiguration('slam_params_file')
    nav2_params_file = LaunchConfiguration('nav2_params_file')

    # 定义节点和包含的launch文件
    load_nodes = GroupAction(
        actions=[
            #裁剪点云
            Node(
                package='cpp_lidar_filter',
                executable='lidar_filter_node',
                name='my_lidar_filter',
                output='screen',
                parameters=[{
                    'input_topic': '/livox/lidar',
                    'output_topic': '/livox/lidar_cropped',
                    # 车身实测 520x520x595mm -> x/y 各 ±0.26
                    # 注意: z 是 base_link 系, 不是地面系. 实测地面在 base_link 约 -0.16~-0.26,
                    # 故下界取 -0.35 覆盖底盘下缘(盒内只有机器人自身, 多挖无害);
                    # 上界 0.60 覆盖 595mm 最高点并留余量
                    'min_x': -0.26, 'max_x': 0.26,
                    'min_y': -0.26, 'max_y': 0.26,
                    'min_z': -0.35, 'max_z': 0.60,
                    'negative': True,   # 挖掉车身
                    'leaf_size': 0.05   # 降采样
                }]
            ),
            # 在 base_link 系去除局部地面，避免 map 系高度变化触发 STVL 误标记。
            # 直接运行源码，新增脚本不依赖 install 中的旧快照。
            ExecuteProcess(
                cmd=["python3", os.path.join(
                    ws_root, "src", "cod_-rm2026_-navigation", "src",
                    "cpp_lidar_filter", "scripts", "ground_filter_node.py"),
                    "--ros-args", "-p", "overhang_filter_enabled:=true",
                    # 2026-10-22: 已确认左侧部件外伸；实测残留回波 x[-.113,.275],
                    # y[.260,.415], z[.181,.397]，局部补裁并留 2~3cm 余量。
                    "-p", "overhang_min:=[-0.14,0.25,0.16]",
                    "-p", "overhang_max:=[0.30,0.44,0.42]"],
                output="screen",
            ),
            #启动LIO里程计
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
            #将3D点云转换为2D
            Node(
                package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
                remappings=[('cloud_in',  '/livox/lidar'),
                            ('scan', '/scan')],
                parameters=[{
                    'target_frame': 'base_link',
                    'transform_tolerance': 0.5,
                    'min_height': 0.1,
                    'max_height': 1.00,
                    'angle_min': -3.1416,  # -M_PI/2
                    'angle_max': 3.1416,  # M_PI/2
                    'angle_increment': 0.0087,  # M_PI/360.0
                    'scan_time': 0.3333,
                    'range_min': 0.5,
                    'range_max': 10.0,
                    'use_inf': True,
                    'inf_epsilon': 1.0
                }],
                name='pointcloud_to_laserscan'
            ),
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[
                    slam_params_file,
                    {'use_sim_time': use_sim_time}
                ],
            ),
            # 自动水平补偿: 启动后用地平面实测 map->odom 并持续发布(替代原静态 map->odom)
            # LIO 世界系每次启动的倾斜角不同(实测见过22.5°/28.8°), 写死补偿会时好时坏
            ExecuteProcess(
                cmd=["python3", os.path.join(ws_root, "auto_level_map.py")],
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
            #把 Nav2 在假坐标系下给出的速度指令，旋转到真实世界坐标系（真实底盘坐标系）下，再转发给串口去执行。
            Node(
                package="fake_vel_transform",
                executable="fake_vel_transform_node",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            ExecuteProcess(
                cmd=["python3", os.path.join(
                    os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
                    "scripts", "footprint_tf_bridge.py"), "--ros-args", "-p",
                    # 底盘 + 左侧外伸部件的保守凸包，必须与车体补裁区域一致。
                    "body_footprint:=[0.30,0.44,0.30,0.25,0.26,-0.26,-0.26,-0.26,-0.26,0.26,-0.14,0.44]"],
                output="screen",
            ),
            Node(
                package="cod_serial_ul26",
                executable="cod_serial",
                output="screen",
                parameters=[{
                    "use_sim_time": use_sim_time,
                    # 用 by-id 固定串口, 避免电控重新枚举成 ttyACM1 后节点仍绑 ttyACM0
                    "port": "/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_374A39593233-if00",
                }],
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
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d',rviz_config_file],
                output='screen',
                condition=IfCondition(use_rviz),
            ),
            IncludeLaunchDescription(
                   PythonLaunchDescriptionSource(
                   os.path.join(get_package_share_directory('cod_bringup'), 'launch', 'auto_save_map.launch.py')
            )
          )
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_slam_params_file,
        declare_nav2_params_file,
        declare_rviz,
        load_nodes
    ])
