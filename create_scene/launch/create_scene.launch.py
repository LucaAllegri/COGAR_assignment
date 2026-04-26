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

##########################
## ADD YOUR BOXES HERE##

# available type: BOX, SPHERE, CYLINDER, CONE

boxes = []
boxes.append(CollisionObject(shape_type = BOX, name="box", size=[0.3, 0.2, 0.2], position=[1.0, 1.0, 1.5], orientation=[0.0,0.0,0.0], color=[1.0, 0.0, 0.0, 1.0], ref_link="world"))

##########################
##########################

##########################
###### DO NOT TOUCH ######

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