from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch引数の定義
    target_distance_arg = DeclareLaunchArgument(
        'target_distance',
        default_value='1.0',
        description='目標距離 [m]'
    )
    
    max_velocity_arg = DeclareLaunchArgument(
        'max_velocity',
        default_value='0.5',
        description='最大速度 [m/s]'
    )
    
    max_acceleration_arg = DeclareLaunchArgument(
        'max_acceleration',
        default_value='0.5',
        description='最大加速度 [m/s^2]'
    )
    
    max_jerk_arg = DeclareLaunchArgument(
        'max_jerk',
        default_value='1.0',
        description='最大ジャーク [m/s^3]'
    )
    
    control_frequency_arg = DeclareLaunchArgument(
        'control_frequency',
        default_value='50.0',
        description='制御周波数 [Hz]'
    )
    
    auto_start_arg = DeclareLaunchArgument(
        'auto_start',
        default_value='false',
        description='自動開始するかどうか'
    )
    
    # ノードの定義
    jerk_trajectory_node = Node(
        package='jerk_trajectory_controller',
        executable='jerk_trajectory_node',
        name='jerk_trajectory_controller',
        output='screen',
        parameters=[{
            'target_distance': LaunchConfiguration('target_distance'),
            'max_velocity': LaunchConfiguration('max_velocity'),
            'max_acceleration': LaunchConfiguration('max_acceleration'),
            'max_jerk': LaunchConfiguration('max_jerk'),
            'control_frequency': LaunchConfiguration('control_frequency'),
            'auto_start': LaunchConfiguration('auto_start'),
        }],
    )
    
    return LaunchDescription([
        target_distance_arg,
        max_velocity_arg,
        max_acceleration_arg,
        max_jerk_arg,
        control_frequency_arg,
        auto_start_arg,
        jerk_trajectory_node,
    ])
