#include <rclcpp/rclcpp.hpp>
#include <moveit_planning/moveit_planning.hpp>

int main(int argc, char **argv){

    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("pick_place_demo");

    internal_planner_settings settings;
    settings.num_attempts = 5;
    settings.replan_attempts = 0;
    settings.planning_time = 5.0;
    settings.constrained_planning_time = 10.0;

    moveit_planning planner(node, "ur_manipulator", settings);

    trajectory_msgs::msg::JointTrajectory trajectory;

    std::vector<double> target = {
        -0.8, -1.3, 1.4, -1.7, -1.57, 0.3
    };

    RCLCPP_INFO(node->get_logger(), "Planning to target configuration...");

    bool ok = planner.plan_trajectory(target, trajectory, {});

    if (ok){
        RCLCPP_INFO(node->get_logger(), "Plan to target succeeded. Executing trajectory...");
        planner.execute_trajectory(trajectory);
    }else{
        RCLCPP_ERROR(node->get_logger(), "Plan to target failed.");
    }

    rclcpp::shutdown();
    return 0;
}