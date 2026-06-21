from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder("ur5e_con_2fg7", package_name="ur5e_2fg7_moveit_config")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    USE_BIN = True
    HARD_SCENE = True

    easy_on_object_position =  [-0.500, 0.150, 0.200]
    easy_on_object_rpy = [3.14,0.0,0.0]
    hard_pre_grasp_position = [-0.500, -0.045, 0.018]
    hard_on_object_position = [-0.500, -0.007, 0.018]
    hard_on_object_rpy = [-1.57,0.0,0.0]
    easy_config_on_table = [0.453786, -1.09956, -1.65806, -1.95477, 1.5708, 0.418879]
    hard_config_on_table = [1.08,-1.18,-2.25,-2.82,1.10,3.14]
    easy_config_on_cabinet = [-0.645772, -1.76278, 1.39626, -4.34587, 1.55334, 2.42601]
    hard_config_on_cabinet = [3.80482, -1.37881, -1.39626, -5.06145, -1.58825, 0.680678]


    params_object = {
        "object_name": "object_box",
        "object_position": [-0.500, 0.150, -0.032],
        "object_size": [0.040, 0.040, 0.100],

        "on_object_position": hard_pre_grasp_position if HARD_SCENE else easy_on_object_position,
        "on_object_rpy": hard_on_object_rpy if HARD_SCENE else easy_on_object_rpy,

        "grasp_position": hard_on_object_position if HARD_SCENE else easy_on_object_position,
    }

    params_basket = {
        "basket_pos": [0.387, 0.000, -0.262],
        "basket_size": [0.500, 0.540, 0.310, 0.010],
    }

    params_robot = {
        "planning_group": "ur_manipulator",
    }

    params_gripper = {
        "gripper_open_position": 0.030,
        "gripper_closed_position": 0.021,
        "gripper_motion_duration": 1.0,
        "gripper_link": "gripper_base_link", # link used to attach the object during simulated grasp
        "use_simulated_contact_grasp": True,
        "simulated_contact_position": 0.0255,
        "simulated_closure_steps": 24,
    }

    configs = {
        "start_config": [0,-1.57,0.0,-1.57,0.0,0.0],
        "config_on_table": hard_config_on_table if HARD_SCENE else easy_config_on_table,
        "config_on_cabinet" : hard_config_on_cabinet if HARD_SCENE else easy_config_on_cabinet
    }


    params_usefull = {
        "use_bin": USE_BIN,
        "hard_scene": HARD_SCENE,
        "num_interpolations": 30 if HARD_SCENE else 20,
        "distance_obj_basket": 0.080,
        "wall_margin": 0.16 if USE_BIN else 0.010,

        "place_contact_object": "basket_bottom" if USE_BIN else "cabinet_lower_body",
        "cabinet_place_pos": [0.247, 0.000, -0.547],
        "cabinet_place_size": [0.780, 0.560, 0.285],   
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