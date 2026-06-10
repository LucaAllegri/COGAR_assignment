#include <rclcpp/rclcpp.hpp>

#include <moveit_planning/moveit_planning.hpp>

#include <std_msgs/msg/float64_multi_array.hpp>

#include <string>
#include <vector>
#include <chrono>


bool planAndExecute(moveit_planning &planner,
                    const rclcpp::Node::SharedPtr &node,
                    const std::vector<double> &target,
                    const std::string &label){
    trajectory_msgs::msg::JointTrajectory trajectory;

    RCLCPP_INFO(node->get_logger(),"Planning to %s configuration...",label.c_str());

    bool plan_ok = planner.plan_trajectory(target, trajectory, {});

    if (!plan_ok){
        RCLCPP_ERROR(node->get_logger(),"Planning to %s failed.",label.c_str());
        return false;
    }

    RCLCPP_INFO(node->get_logger(),"Executing %s trajectory...",label.c_str());

    bool exec_ok = planner.execute_trajectory(trajectory);

    if (!exec_ok){
        RCLCPP_ERROR(node->get_logger(),"Execution of %s trajectory failed.",label.c_str());
        return false;
    }

    RCLCPP_INFO(node->get_logger(),"%s trajectory completed.",label.c_str());

    return true;
}


void moveGripper(const rclcpp::Node::SharedPtr &node,
                double position,
                double duration_sec){
    
    auto publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>("/gripper_position_controller/commands",10);

    std_msgs::msg::Float64MultiArray msg;
    msg.data = {position, position};

    RCLCPP_INFO(node->get_logger(),"Commanding 2FG7 gripper position: %.4f",position);

    rclcpp::Time start_time = node->now();

    while (rclcpp::ok() && (node->now() - start_time).seconds() < duration_sec){
        publisher->publish(msg);
        rclcpp::spin_some(node);
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
}


bool simulatedForceGrasp(const rclcpp::Node::SharedPtr &node,
                        double target_force,
                        double force_threshold){
    RCLCPP_INFO(node->get_logger(),"Closing 2FG7 gripper in simulated force mode...");

    RCLCPP_INFO(node->get_logger(),"Target force: %.2f N",target_force);

    RCLCPP_INFO(node->get_logger(),"Force threshold: %.2f N",force_threshold);

    if (target_force >= force_threshold){
        RCLCPP_INFO(node->get_logger(),"Simulated force threshold reached. Grasp accepted.");
        return true;
    }

    RCLCPP_WARN(node->get_logger(),"Simulated force threshold not reached. Grasp rejected.");

    return false;
}


int main(int argc, char **argv){
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("test_pick_node");

    std::string planning_group = node->declare_parameter<std::string>("planning_group","ur_manipulator");

    std::string object_name = node->declare_parameter<std::string>("object_name","object_box");

    std::string gripper_link = node->declare_parameter<std::string>("gripper_link","gripper_base_link");

    double target_force = node->declare_parameter<double>("target_force",40.0);

    double force_threshold = node->declare_parameter<double>("force_threshold",30.0);

    double gripper_open_position = node->declare_parameter<double>("gripper_open_position",0.030);

    double gripper_closed_position = node->declare_parameter<double>("gripper_closed_position",0.005);

    double gripper_motion_duration = node->declare_parameter<double>("gripper_motion_duration",1.0);

    int num_repetitions = node->declare_parameter<int>("num_repetitions",1);

    std::vector<double> configuration_home = node->declare_parameter<std::vector<double>>("configuration_home",std::vector<double>{0.0, -1.57, 1.57, -1.57, -1.57, 0.0});

    std::vector<double> configuration_pre_pick = node->declare_parameter<std::vector<double>>("configuration_pre_pick",std::vector<double>{-0.8, -1.3, 1.4, -1.7, -1.57, 0.3});

    std::vector<double> configuration_lift = node->declare_parameter<std::vector<double>>("configuration_lift",std::vector<double>{-0.8, -1.20, 1.35, -1.65, -1.57, 0.3});

    internal_planner_settings settings;
    settings.num_attempts = 5;
    settings.replan_attempts = 0;
    settings.planning_time = 5.0;
    settings.constrained_planning_time = 10.0;

    moveit_planning planner(node, planning_group, settings);

    int successful_trials = 0;

    for (int trial = 0; trial < num_repetitions; trial++){
        RCLCPP_INFO(node->get_logger(), "==============================");
        RCLCPP_INFO(node->get_logger(),"Starting pick trial %d/%d",trial + 1,num_repetitions);
        RCLCPP_INFO(node->get_logger(), "==============================");

        bool ok_home = planAndExecute(planner,node,configuration_home,"home");

        if (!ok_home){
            RCLCPP_ERROR(node->get_logger(),"Trial %d failed at home motion.",trial + 1);
            continue;
        }

        RCLCPP_INFO(node->get_logger(),"Opening gripper before approaching the object...");

        moveGripper(node,gripper_open_position,gripper_motion_duration);

        bool ok_pre_pick = planAndExecute(planner,node,configuration_pre_pick,"pre-pick");

        if (!ok_pre_pick){
            RCLCPP_ERROR(node->get_logger(),"Trial %d failed at pre-pick motion.",trial + 1);
            continue;
        }

        bool grasp_ok = simulatedForceGrasp(node,target_force,force_threshold);

        if (!grasp_ok){
            RCLCPP_ERROR(node->get_logger(),"Trial %d failed: simulated force grasp rejected.",trial + 1);
            continue;
        }

        RCLCPP_INFO(node->get_logger(),"Closing gripper on object...");

        moveGripper(node,gripper_closed_position,gripper_motion_duration);

        RCLCPP_INFO(node->get_logger(),"Attaching object '%s' to gripper link '%s'...",object_name.c_str(),gripper_link.c_str());

        planner.attach_object(object_name, gripper_link);

        bool ok_lift = planAndExecute(planner,node,configuration_lift,"lift");

        if (!ok_lift){
            RCLCPP_ERROR(node->get_logger(),"Trial %d failed at lift motion.",trial + 1);

            planner.detach_object(object_name);
            continue;
        }

        RCLCPP_INFO(node->get_logger(),"Pick trial %d completed successfully.",trial + 1);

        successful_trials++;
    }

    RCLCPP_INFO(node->get_logger(), "==============================");
    RCLCPP_INFO(node->get_logger(), "Pick validation completed.");
    RCLCPP_INFO(node->get_logger(),"Successful trials: %d/%d",successful_trials,num_repetitions
    );
    RCLCPP_INFO(node->get_logger(), "==============================");

    rclcpp::shutdown();
    return 0;
}