from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('zedx_pure_pursuit'),
        'config',
        'pure_pursuit_params.yaml'
    )
    print(f"Path to Config: {config_path}")
    return LaunchDescription(
        Node(
            package='zed_wrapper',
            executable='zed_camera_node',
            name='zed_camera_node',
            output='screen',
            parameters=[{
                'camera_model': 'zedx',
                'positional_tracking': True,
                'publish_tf': True,
                'two_d_mode' : True
            }]
        ),
        Node(
            package='zedx_pure_pursuit',  # your Python bridge node package
            executable='zed_localization_bridge',
            name='zed_localization_bridge',
            output='screen',
        ),

        Node(
            package='zedx_pure_pursuit',
            executable='pure_pursuit_node',
            name='pure_pursuit_node',
            output='screen',
            parameters=[config_path],
        ),

    )