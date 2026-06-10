import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch_ros.actions import Node

BOX = 1
SPHERE = 2
CYLINDER = 3
CONE = 4


class CollisionObject:
    def __init__(self,shape_type=BOX,name="box",size=[0.1, 0.1, 0.1],position=[0.0, 0.0, 0.0],orientation=[0.0, 0.0, 0.0],color=[1.0, 0.0, 0.0, 1.0],ref_link="base_link",) -> None:
        self.shape_type = shape_type
        self.name = name
        self.size = size
        self.position = position
        self.orientation = orientation
        self.color = color
        self.ref_link = ref_link

    def build_dict(self, suffix) -> dict:
        return {
            "shape_type" + suffix: self.shape_type,
            "name" + suffix: self.name,
            "size" + suffix: self.size,
            "position" + suffix: self.position,
            "orientation" + suffix: self.orientation,
            "color" + suffix: self.color,
            "ref_link" + suffix: self.ref_link,
        }


boxes = []

# COLORI PEZZI
cabinet_color = [0.16, 0.18, 0.19, 1.0]   
tower_color = [0.64, 0.64, 0.64, 1.0]
table_color = [1, 0.20, 0.20, 1.0]
basket_color = [0.70, 0.53, 0.40, 1.0]
#------------

# corpo basso cabinet
boxes.append(CollisionObject(shape_type=BOX,name="cabinet_lower_body",size=[0.780, 0.560, 0.285],position=[0.247, 0.000, -0.547],orientation=[0.0, 0.0, 0.0],color=cabinet_color,ref_link="base_link",))
#------------

# Torre superiore su cui è montato il robot
boxes.append(CollisionObject(shape_type=BOX,name="cabinet_tower",size=[0.200, 0.250, 0.5469999],position=[0.000, 0.000, -0.547],orientation=[0.0, 0.0, 0.0],color=tower_color,ref_link="base_link",))
# BordI attorno alla torre
boxes.append(CollisionObject(shape_type=BOX,name="cabinet_boundary_1",size=[0.280, 0.005, 0.427],position=[0.000, -0.2775, -0.547],orientation=[0.0, 0.0, 0.0],color=cabinet_color,ref_link="base_link",))
boxes.append(CollisionObject(shape_type=BOX,name="cabinet_boundary_2",size=[0.280, 0.005, 0.427],position=[0.000, 0.2775, -0.547],orientation=[0.0, 0.0, 0.0],color=cabinet_color,ref_link="base_link",))
boxes.append(CollisionObject(shape_type=BOX,name="cabinet_boundary_3",size=[0.005, 0.560, 0.427],position=[-0.143, 0.000, -0.547],orientation=[0.0, 0.0, 0.0],color=cabinet_color,ref_link="base_link",))
#------------

# Oggetto da graspare
boxes.append(CollisionObject(shape_type=BOX,name="object_box",size=[0.060, 0.060, 0.120],position=[-0.500, 0.000, -0.142],orientation=[0.0, 0.0, 0.0],color=[0.0, 0.45, 1.0, 1.0],ref_link="base_link",))
#------------

# Piano del tavolo
boxes.append(CollisionObject(shape_type=BOX,name="opposite_table_top",size=[0.500, 0.560, 0.050],position=[-0.500, 0.000, -0.192],orientation=[0.0, 0.0, 0.0],color=table_color,ref_link="base_link",))
# Gambe del tavolo
boxes.append(CollisionObject(shape_type=BOX,name="opposite_table_leg_1",size=[0.040, 0.040, 0.355],position=[-0.700, -0.235, -0.547],orientation=[0.0, 0.0, 0.0],color=table_color,ref_link="base_link",))
boxes.append(CollisionObject(shape_type=BOX,name="opposite_table_leg_2",size=[0.040, 0.040, 0.355],position=[-0.700, 0.235, -0.547],orientation=[0.0, 0.0, 0.0],color=table_color,ref_link="base_link",))
boxes.append(CollisionObject(shape_type=BOX,name="opposite_table_leg_3",size=[0.040, 0.040, 0.355],position=[-0.300, -0.235, -0.547],orientation=[0.0, 0.0, 0.0],color=table_color,ref_link="base_link",))
boxes.append(CollisionObject(shape_type=BOX,name="opposite_table_leg_4",size=[0.040, 0.040, 0.355],position=[-0.300, 0.235, -0.547],orientation=[0.0, 0.0, 0.0],color=table_color,ref_link="base_link",))
#------------

# OSTACOLO
boxes.append(CollisionObject(shape_type=CYLINDER,name="obstacle_1",size=[0.8, 0.03],position=[-0.400, 0.000, 0.24],orientation=[0.0, 0.0, 0.0],color=[1.0, 1.0, 0.0, 1.0],ref_link="base_link",))
boxes.append(CollisionObject(shape_type=CYLINDER,name="obstacle_2",size=[0.8, 0.03],position=[-0.300, 0.000, 0.24],orientation=[0.0, 0.0, 0.0],color=[1.0, 1.0, 0.0, 1.0],ref_link="base_link",))
#------------

# Cestino sul cabinet fondo
basket_x = 0.500
basket_y = 0.540
basket_z = 0.310
basket_t = 0.010
basket_pos_x = 0.387
basket_pos_y = 0.000
basket_pos_z = -0.262
boxes.append(CollisionObject(shape_type=BOX,name="basket_bottom",size=[basket_x, basket_y, basket_t],position=[basket_pos_x, basket_pos_y, basket_pos_z],orientation=[0.0, 0.0, 0.0],color=basket_color,ref_link="base_link",))
# lati 
boxes.append(CollisionObject(shape_type=BOX,name="basket_front_wall",size=[basket_x, basket_t, basket_z - basket_t],position=[basket_pos_x, basket_pos_y - basket_y/2 + basket_t/2, basket_pos_z + basket_t],orientation=[0.0, 0.0, 0.0],color=basket_color,ref_link="base_link",))
boxes.append(CollisionObject(shape_type=BOX,name="basket_back_wall",size=[basket_x, basket_t, basket_z - basket_t],position=[basket_pos_x, basket_pos_y + basket_y/2 - basket_t/2, basket_pos_z + basket_t],orientation=[0.0, 0.0, 0.0],color=basket_color,ref_link="base_link",))
boxes.append(CollisionObject(shape_type=BOX,name="basket_left_wall",size=[basket_t, basket_y, basket_z - basket_t],position=[basket_pos_x - basket_x/2 + basket_t/2, basket_pos_y, basket_pos_z + basket_t],orientation=[0.0, 0.0, 0.0],color=basket_color,ref_link="base_link",))
boxes.append(CollisionObject(shape_type=BOX,name="basket_right_wall",size=[basket_t, basket_y, basket_z - basket_t],position=[basket_pos_x + basket_x/2 - basket_t/2, basket_pos_y, basket_pos_z + basket_t],orientation=[0.0, 0.0, 0.0],color=basket_color,ref_link="base_link",))


def generate_launch_description():

    params = {"num_boxes": len(boxes)}

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
            parameters=[params],
        )
    )

    return LaunchDescription(nodes)