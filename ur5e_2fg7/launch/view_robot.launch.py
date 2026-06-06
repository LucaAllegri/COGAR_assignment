import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_ur5e_2fg7 = get_package_share_directory('ur5e_2fg7')
    
    # Leggiamo il file URDF finale statico
    urdf_file = os.path.join(pkg_ur5e_2fg7, 'urdf', 'ur5e_2fg7_final.urdf')
    
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    rviz_config_file = os.path.join(pkg_ur5e_2fg7, 'config', 'view_robot.rviz')

    return LaunchDescription([
        # Nodo 1: Robot State Publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_desc}]
        ),
        
        # Nodo 2: Joint State Publisher GUI con posizioni iniziali pre-impostate
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            parameters=[{
                'zeros': {
                    'shoulder_pan_joint': 0.0,
                    'shoulder_lift_joint': -1.5708,
                    'elbow_joint': 0.0,
                    'wrist_1_joint': -1.5708,
                    'wrist_2_joint': 0.0,
                    'wrist_3_joint': 0.0,
                    'gripper_gripper_joint': 0.000
                }
            }]
        ),
        
        # Nodo 3: RViz2
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=['-d', rviz_config_file]
        )
    ])