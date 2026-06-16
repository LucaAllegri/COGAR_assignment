from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder("ur5e_con_2fg7", package_name="ur5e_2fg7_moveit_config")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    params_object = {
        "object_name": "object_box",
        "object_position": [-0.500, -0.100, -0.032],
        "object_size": [0.040, 0.040, 0.100],

        "on_object_position": [-0.500, -0.100, 0.200],
        "on_object_rpy": [3.14,0.0,0.0],
    }

    params_basket = {
        "basket_pos": [0.387, 0.000, -0.262],
        "basket_size": [0.500, 0.540, 0.310, 0.010],
        "basket_contact_object": "basket_bottom",
    }

    params_robot = {
        "planning_group": "ur_manipulator",

        # Simulated force-grasp threshold
        "target_force": 40.0,
        "force_threshold": 30.0,
    }

    params_gripper = {
        "gripper_open_position": 0.030,
        "gripper_closed_position": 0.026,
        "gripper_motion_duration": 1.0,
        "gripper_link": "gripper_base_link", # link used to attach the object during simulated grasp
    }

    configs = {
        "start_config": [0,-1.57,0.0,-1.57,0.0,0.0],
        "config_on_table": [0.453786, -1.09956, -1.65806, -1.95477, 1.5708, 0.418879],
        "config_on_cabinet" : [3.76991, -0.907571, -1.5708, 4.10152, 1.5708, 0.645772],
    }


    params_usefull = {
        "use_bin": True,
        "num_interpolations": 20,
        "hard_scene": False,
        "distance_obj_basket": 0.080,  #try 0.005 / 0.020 / 0.050 / 0.080
        "wall_margin": 0.16,
    }

    return LaunchDescription([
        Node(
            package="tasks_demo",
            executable="pick",
            name="test_pick",
            output="screen",
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                params_robot,
                params_object,
                params_usefull,
                configs,
                params_gripper,
                params_basket,
            ],
        )
    ])