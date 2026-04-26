#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <map>
#include <string>
#include <vector>

class UR5eSimPlanner : public rclcpp::Node{
    public:

        explicit UR5eSimPlanner(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()): Node("ur5e_sim_planner", options){
            RCLCPP_INFO(get_logger(), "UR5eSimPlanner inizializzato.");
        }

        // ── Joint Space Motion ─────────────────────────────────────────────
        bool moveJoints(const std::map<std::string, double>& joint_positions){
            moveit::planning_interface::MoveGroupInterface mgi(shared_from_this(), "ur_manipulator");

            mgi.setStartStateToCurrentState();
            mgi.setMaxVelocityScalingFactor(0.3);
            mgi.setMaxAccelerationScalingFactor(0.1);
            mgi.setPlanningTime(5.0);
            mgi.setNumPlanningAttempts(10);

            mgi.setJointValueTarget(joint_positions);

            moveit::planning_interface::MoveGroupInterface::Plan plan;
            bool ok = (mgi.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

            if (ok) {
                RCLCPP_INFO(get_logger(), "[JOINT] Piano trovato, eseguo...");
                mgi.execute(plan);
                RCLCPP_INFO(get_logger(), "[JOINT] Movimento completato!");
            } else {
                RCLCPP_ERROR(get_logger(), "[JOINT] Planning FALLITO!");
            }

            return ok;
        }

        // ── Cartesian Pose Motion ──────────────────────────────────────────
        bool movePose(double x, double y, double z,double roll = M_PI, double pitch = 0.0, double yaw = 0.0){
            moveit::planning_interface::MoveGroupInterface mgi(shared_from_this(), "ur_manipulator");

            mgi.setMaxVelocityScalingFactor(0.2);
            mgi.setMaxAccelerationScalingFactor(0.1);
            mgi.setPlanningTime(5.0);
            mgi.setNumPlanningAttempts(10);
            mgi.setEndEffectorLink("tool0");
            mgi.setPoseReferenceFrame("base_link");

            // Costruisci quaternion da RPY
            tf2::Quaternion q;
            q.setRPY(roll, pitch, yaw);

            geometry_msgs::msg::Pose target;
            target.position.x = x;
            target.position.y = y;
            target.position.z = z;
            target.orientation = tf2::toMsg(q);

            mgi.setPoseTarget(target);

            moveit::planning_interface::MoveGroupInterface::Plan plan;
            bool ok = (mgi.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

            if (ok) {
                RCLCPP_INFO(get_logger(),"[POSE] Piano trovato verso (%.3f, %.3f, %.3f), eseguo...",x, y, z);
                mgi.execute(plan);
                RCLCPP_INFO(get_logger(), "[POSE] Movimento completato!");
            } else {
                RCLCPP_ERROR(get_logger(), "[POSE] Planning FALLITO!");
            }

            return ok;
        }

        // ── Cartesian Path (linea retta) ───────────────────────────────────
        bool moveCartesianPath(const std::vector<geometry_msgs::msg::Pose>& waypoints){
            moveit::planning_interface::MoveGroupInterface mgi(shared_from_this(), "ur_manipulator");

            mgi.setMaxVelocityScalingFactor(0.1);   // lento per path cartesiano
            mgi.setMaxAccelerationScalingFactor(0.1);
            mgi.setEndEffectorLink("tool0");
            mgi.setPoseReferenceFrame("base_link");

            moveit_msgs::msg::RobotTrajectory trajectory;
            const double eef_step    = 0.01;   // 1cm di risoluzione
            const double jump_thresh = 0.0;    // disabilita jump threshold

            double fraction = mgi.computeCartesianPath(
            waypoints, eef_step, jump_thresh, trajectory);

            RCLCPP_INFO(get_logger(),"[CARTESIAN PATH] Pianificato %.1f%% del percorso", fraction * 100.0);

            if (fraction > 0.95) {   // almeno 95% del path pianificato
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                plan.trajectory_ = trajectory;
                mgi.execute(plan);
                RCLCPP_INFO(get_logger(), "[CARTESIAN PATH] Eseguito!");
                return true;
            } else {
                RCLCPP_ERROR(get_logger(), "[CARTESIAN PATH] Path incompleto!");
                return false;
            }   
        }
};


// ── Posizioni predefinite ──────────────────────────────────────────────
std::map<std::string, double> makeHome(){
    return {
        {"shoulder_pan_joint",   0.0},
        {"shoulder_lift_joint", -1.5707},
        {"elbow_joint",          1.5707},
        {"wrist_1_joint",       -1.5707},
        {"wrist_2_joint",       -1.5707},
        {"wrist_3_joint",        0.0},
    };
}

std::map<std::string, double> makeCanReady(){
    return {
        {"shoulder_pan_joint",   0.0},
        {"shoulder_lift_joint", -1.2},
        {"elbow_joint",          1.0},
        {"wrist_1_joint",       -1.3707},
        {"wrist_2_joint",       -1.5707},
        {"wrist_3_joint",        0.0},
    };
}


int main(int argc, char** argv){
    rclcpp::init(argc, argv);

    // NodeOptions che eredita TUTTI i parametri remapped
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<UR5eSimPlanner>(node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor]() { executor.spin(); });

    std::this_thread::sleep_for(std::chrono::seconds(2));


    RCLCPP_INFO(rclcpp::get_logger("motion_planner"), "Waiting for MoveIt...");
    rclcpp::sleep_for(std::chrono::seconds(5));
    node->moveJoints(makeHome());

    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 0;
}