import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from enum import IntEnum

##########################
###### DO NOT TOUCH ######

BOX = 1
SPHERE = 2
CYLINDER = 3
CONE = 4

class CollisionObject:
    def __init__(self, shape_type = BOX, name = "box", size = [0.1,0.1,0.1], position = [0.0,0.0,0.0], orientation = [0.0,0.0,0.0], color = [1.0,0.0,0.0,0.0], ref_link = "world") -> None:
        self.shape_type = shape_type
        self.name = name
        self.size = size
        self.position = position
        self.orientation = orientation
        self.color = color
        self.ref_link = ref_link

    def build_dict(self, suffix) -> dict:
        return {
            "shape_type" + suffix : self.shape_type,
            "name" + suffix : self.name,
            "size" + suffix : self.size,
            "position" + suffix : self.position,
            "orientation" + suffix : self.orientation,
            "color" + suffix : self.color,
            "ref_link" + suffix : self.ref_link
        }

boxes = []

# CABINET
boxes.append(CollisionObject(shape_type=BOX, name="cabinet", size=[0.5, 0.5, 0.40], position=[-0.17, 0.00, -0.4001], orientation=[0.0, 0.0, 0.0], color=[0.45, 0.30, 0.18, 1.0], ref_link="base_link"))

# BACK WALL
boxes.append(CollisionObject(shape_type=BOX, name="back_wall", size=[0.05, 0.8, 1.0], position=[-0.445, -0.15, -0.4], orientation=[0.0, 0.0, 0.0], color=[0.65, 0.65, 0.65, 1.0], ref_link="base_link"))

# RIGHT WALL
boxes.append(CollisionObject(shape_type=BOX, name="right_wall", size=[0.55, 0.05, 1.0], position=[-0.195, 0.4, -0.4], orientation=[0.0, 0.0, 0.0], color=[0.65, 0.65, 0.65, 1.0], ref_link="base_link"))

# TABLE RED
boxes.append(CollisionObject(shape_type=BOX, name="table_red", size=[0.45, 0.50, 0.35], position=[0.35, 0.00, -0.3501], orientation=[0.0, 0.0, 0.0], color=[1.0, 0.0, 0.0, 1.0], ref_link="base_link"))

# TABLE BLUE
boxes.append(CollisionObject(shape_type=BOX, name="table_blue", size=[0.50, 0.45, 0.35], position=[-0.17, -0.515, -0.3501], orientation=[0.0, 0.0, 0.0], color=[0.0, 0.0, 1.0, 1.0], ref_link="base_link"))

# LEFT WALL
#boxes.append(CollisionObject(shape_type=BOX, name="left_wall", size=[0.70, 0.05, 0.70], position=[-0.45, -0.425, 0.00], orientation=[0.0, 0.0, 0.0], color=[0.65, 0.65, 0.65, 1.0], ref_link="base_link"))

# TOP WALL
#boxes.append(CollisionObject(shape_type=BOX, name="top_wall", size=[0.70, 0.80, 0.05], position=[-0.45, 0.00, 0.70], orientation=[0.0, 0.0, 0.0], color=[0.55, 0.55, 0.55, 1.0], ref_link="base_link"))

# PARALLELEPIPED TO GRASP
boxes.append(CollisionObject(shape_type=BOX, name="stick", size=[0.05, 0.05, 1.0], position=[0.35, -0.5, -0.3501], orientation=[0.0, 0.0, 0.0], color=[0.0, 1.0, 0.0, 1.0], ref_link="base_link"))


def generate_launch_description():

    params = {"num_boxes":len(boxes)}

    suffix = 0
    for b in boxes:
        params = params | b.build_dict("_" + str(suffix))
        suffix += 1

    nodes = []
    nodes.append(
            Node(
                package="create_scene",
                executable="scene",
                output="screen",
                parameters=[
                    params
                ],
            )
        )

    return LaunchDescription(nodes)