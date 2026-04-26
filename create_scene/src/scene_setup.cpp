#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/object_color.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>

std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
std::vector<moveit_msgs::msg::ObjectColor> colors;

void create_object(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface,
                   const int object_shape_type,
                   const std::string &name,
                   const std::vector<double> size,
                   const std::vector<double> position,
                   const std::vector<double> orientation,
                   const std::vector<double> color,
                   const std::string ref_link = "world"){

    moveit_msgs::msg::CollisionObject collision_object_msgs;

    collision_object_msgs.header.frame_id = ref_link;
    collision_object_msgs.id = name;
    collision_object_msgs.primitives.resize(1);
    collision_object_msgs.primitives[0].type = object_shape_type;
    collision_object_msgs.primitives[0].dimensions.resize(3);
    collision_object_msgs.primitives[0].dimensions[0] = size[0];
    collision_object_msgs.primitives[0].dimensions[1] = size[1];
    collision_object_msgs.primitives[0].dimensions[2] = size[2];
    collision_object_msgs.primitive_poses.resize(1);
    collision_object_msgs.primitive_poses[0].position.x = position[0];
    collision_object_msgs.primitive_poses[0].position.y = position[1];
    collision_object_msgs.primitive_poses[0].position.z = position[2] + size[2] / 2;
    tf2::Quaternion quat;
    quat.setRPY(orientation[0], orientation[1], orientation[2]);
    collision_object_msgs.primitive_poses[0].orientation.x = quat.x();
    collision_object_msgs.primitive_poses[0].orientation.y = quat.y();
    collision_object_msgs.primitive_poses[0].orientation.z = quat.z();
    collision_object_msgs.primitive_poses[0].orientation.w = quat.w();
    collision_object_msgs.operation = collision_object_msgs.ADD;

    collision_objects.push_back(collision_object_msgs);

    moveit_msgs::msg::ObjectColor color_msgs;
    color_msgs.color.r = color[0];
    color_msgs.color.g = color[1];
    color_msgs.color.b = color[2];
    color_msgs.color.a = color[3];
    colors.push_back(color_msgs);
}


int main(int argc, char** argv){

    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("create_objects_scene");
    
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    auto num_boxes = node->declare_parameter<int>("num_boxes", 0);

    for (int i = 0; i < num_boxes; i++){
        auto shape_type = node->declare_parameter<int>("shape_type_" + std::to_string(i), shape_msgs::msg::SolidPrimitive::BOX);
        auto name = node->declare_parameter<std::string>("name_" + std::to_string(i), "box");
        auto size = node->declare_parameter<std::vector<double>>("size_" + std::to_string(i), std::vector<double>{0.1, 0.1, 0.1});
        auto position = node->declare_parameter<std::vector<double>>("position_" + std::to_string(i), std::vector<double>{0.0, 0.0, 0.0});
        auto orientation = node->declare_parameter<std::vector<double>>("orientation_" + std::to_string(i), std::vector<double>{0.0, 0.0, 0.0});
        auto color = node->declare_parameter<std::vector<double>>("color_" + std::to_string(i), std::vector<double>{1.0, 0.0, 0.0, 1.0});
        auto ref_link = node->declare_parameter<std::string>("ref_link_" + std::to_string(i), "world");
        create_object(planning_scene_interface, shape_type, name, size, position, orientation, color, ref_link);
    }

    planning_scene_interface.applyCollisionObjects(collision_objects, colors);

    rclcpp::sleep_for(std::chrono::seconds(2));
    rclcpp::shutdown();
    return 0;
}