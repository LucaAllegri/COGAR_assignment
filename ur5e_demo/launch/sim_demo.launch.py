from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro


def generate_launch_description():

    # Processa xacro → stringa URDF
    urdf_path = os.path.join(
        get_package_share_directory('ur_description'),
        'urdf', 'ur.urdf.xacro'
    )
    srdf_path = os.path.join(
        get_package_share_directory('ur_moveit_config'),
        'srdf', 'ur.srdf.xacro'
    )

    robot_description = xacro.process_file(
        urdf_path,
        mappings={'ur_type': 'ur5e', 'name': 'ur'}
    ).toxml()

    robot_description_semantic = xacro.process_file(
        srdf_path,
        mappings={'name': 'ur'}
    ).toxml()

    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ur_moveit_config'),
                'launch', 'ur_moveit.launch.py'
            )
        ),
        launch_arguments={
            'ur_type':           'ur5e',
            'use_fake_hardware': 'true',
            'launch_rviz':       'true',
        }.items()
    )

    create_scene_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('create_scene'),
                'launch',
                'create_scene.launch.py'
            )
        )
    )

    motion_node = TimerAction(
    period=20.0,
    actions=[Node(
        package='ur5e_demo',
        executable='motion_planner',
        name='motion_planner',
        output='screen',
        parameters=[{
            'robot_description':          robot_description,
            'robot_description_semantic': robot_description_semantic,
        }],
    )]
)

    return LaunchDescription([
        ur_moveit_launch,
        create_scene_launch,
        motion_node,
    ])