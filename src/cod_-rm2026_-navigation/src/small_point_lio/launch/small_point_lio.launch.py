from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration,PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    #静态TF参数声明，大yaw固定，如果用动态yaw，注释掉这一段
    mount_x = DeclareLaunchArgument('mount_x', default_value='0.05289') 
    mount_y = DeclareLaunchArgument('mount_y', default_value='0.15191')   
    mount_z = DeclareLaunchArgument('mount_z', default_value='0.15691')    
    mount_roll  = DeclareLaunchArgument('mount_roll', default_value='0.0')
    mount_pitch = DeclareLaunchArgument('mount_pitch', default_value='0.3839')  #pitch偏移22°≈0.3839 rad
    mount_yaw   = DeclareLaunchArgument('mount_yaw', default_value='0.0')       #与大yaw同步，待电控传回
    small_point_lio_node = Node(
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
    )
    #----装位TF：从上面参数取值，不写死----
    static_base_link_to_livox_frame = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x" , LaunchConfiguration("mount_x"),
            "--y" , LaunchConfiguration("mount_y"),
            "--z" , LaunchConfiguration("mount_z"),
            "--roll"  , LaunchConfiguration("mount_roll"),
            "--pitch" , LaunchConfiguration("mount_pitch"),
            "--yaw"   , LaunchConfiguration("mount_yaw"),
            "--frame-id","base_link",
            "--child-frame-id","livox_frame",
        ],
    )
    #动态TF广播器节点，订阅/yaw话题，发布大yaw的TF
    yaw_tf_broadcaster_node = Node(
    package='small_point_lio',
    executable='yaw_tf_broadcaster',
    name='yaw_tf_broadcaster',
    output='screen',
    parameters=[{
        'mount_x': 0.05289,
        'mount_y': 0.15191,
        'mount_z': 0.15691,
        'mount_roll': 0.0,
        'mount_pitch': 0.3839,  # 22°
    }],
)
    return LaunchDescription([small_point_lio_node, static_base_link_to_livox_frame, yaw_tf_broadcaster_node, mount_x, mount_y, mount_z, mount_roll, mount_pitch, mount_yaw])
