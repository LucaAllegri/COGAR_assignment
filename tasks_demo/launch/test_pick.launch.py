from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder("ur5e_con_2fg7", package_name="ur5e_2fg7_moveit_config")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    params_robot = {
        "planning_group": "ur_manipulator",

        # Object already present in create_scene_2.launch.py
        "object_name": "object_box",

        # Link used to attach the object during simulated grasp
        "gripper_link": "gripper_base_link",

        # Simulated force-grasp threshold
        "target_force": 40.0,
        "force_threshold": 30.0,

        "gripper_open_position": 0.030,
        "gripper_closed_position": 0.005,
        "gripper_motion_duration": 1.0,

        # Number of repetitions for reliability test
        "num_repetitions": 1,

        # Joint-space configurations for UR5e
        "configuration_home": [
            -1.55334, -1.97222, 1.88496, -1.48353, -1.5708, -1.55334
        ],

        "configuration_pre_pick": [
            -0.331613, -1.67552, 2.18166, -2.0944, -1.5708, -0.331613
        ],

        "configuration_lift": [
            -0.331613, -1.67552, 2.18166, -2.0944, -1.5708, -0.331613
        ],
    }

    return LaunchDescription([
        Node(
            package="tasks_demo",
            executable="pick",
            name="test_pick_node",
            output="screen",
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                params_robot,
            ],
        )
    ])