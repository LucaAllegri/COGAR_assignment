#include <rclcpp/rclcpp.hpp>

#include <moveit_planning/moveit_planning.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <std_msgs/msg/float64_multi_array.hpp>

#include <string>
#include <vector>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <cmath>

bool contactWithDesiredObjectOnly(const std::vector<std::pair<std::string, std::string>> &contact_pairs, const std::string &contact_object,const std::string &object_name){
    bool desired_contact_found = false;

    for (const auto &pair : contact_pairs) {

        std::cout << "Contact pair: " << pair.first << " - " << pair.second << std::endl;

        bool involves_contact_object = (pair.first == contact_object || pair.second == contact_object);

        bool involves_grasped_object = (pair.first == object_name || pair.second == object_name);

        if (involves_contact_object && involves_grasped_object) {
            desired_contact_found = true;
        } else {
            return false; //c'è collisione
        }
    }

    return desired_contact_found;
}


bool plan_vertical_cartesian_until_contact_from_state(const Eigen::Affine3d &start_pose,const std::vector<double> &start_config,double target_search_z,const std::string &contact_object,const std::string &object_name,double eef_step,double penetration,trajectory_msgs::msg::JointTrajectory &trajectory,moveit_planning &robot_planning){
    if (start_config.size() != 6) {
        std::cout << "Invalid start_config size: " << start_config.size() << std::endl;
        return false;
    }

    double start_z = start_pose.translation().z();
    double dz_total = target_search_z - start_z;

    if (dz_total >= 0.0) {
        std::cout << "Target z is not below start z. No vertical descent needed." << std::endl;
        return false;
    }

    int n_steps = static_cast<int>(std::ceil(std::abs(dz_total) / eef_step));

    if (n_steps < 1) {
        n_steps = 1;
    }

    trajectory.joint_names = {
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint"
    };

    trajectory.points.clear();

    trajectory_msgs::msg::JointTrajectoryPoint start_point;
    start_point.positions = start_config;
    start_point.time_from_start = rclcpp::Duration::from_seconds(0.0);
    trajectory.points.push_back(start_point);

    std::vector<double> previous_config = start_config;

    double accumulated_penetration = 0.0;
    bool contact_found = false;

    for (int step = 1; step <= n_steps; step++) {

        double alpha = static_cast<double>(step) / static_cast<double>(n_steps);

        Eigen::Affine3d waypoint_pose = start_pose;
        waypoint_pose.translation().z() = start_z + alpha * dz_total;

        std::cout << "Checking vertical waypoint "<< step << "/" << n_steps<< " z = " << waypoint_pose.translation().z()<< std::endl;

        std::vector<std::vector<double>> ik_solutions;
        robot_planning.inverse_kinematics(waypoint_pose,ik_solutions,previous_config);

        if (ik_solutions.empty()) {
            std::cout << "No IK solution for vertical waypoint." << std::endl;
            return false;
        }

        bool waypoint_accepted = false;

        for (size_t i = 0; i < ik_solutions.size(); i++) {

            std::vector<double> candidate_config = ik_solutions[i];

            std::vector<std::pair<std::string, std::string>> contact_pairs;

            bool state_valid = robot_planning.check_state_validity(candidate_config,contact_pairs);

            if (state_valid) {

                trajectory_msgs::msg::JointTrajectoryPoint point;
                point.positions = candidate_config;
                point.time_from_start = rclcpp::Duration::from_seconds(0.25 * static_cast<double>(trajectory.points.size()));

                trajectory.points.push_back(point);

                previous_config = candidate_config;
                waypoint_accepted = true;
                break;
            }

            // Se lo stato non è valido, controllo se è la collisione desiderata:
            // object_box - basket_bottom
            bool desired_contact = contactWithDesiredObjectOnly(contact_pairs,contact_object,object_name);

            if (desired_contact) {

                std::cout << "Desired contact detected with " << contact_object << std::endl;

                trajectory_msgs::msg::JointTrajectoryPoint point;
                point.positions = candidate_config;
                point.time_from_start = rclcpp::Duration::from_seconds(0.25 * static_cast<double>(trajectory.points.size()));

                trajectory.points.push_back(point);

                previous_config = candidate_config;
                waypoint_accepted = true;
                contact_found = true;

                break;

            } else {
                std::cout << "Invalid collision for this waypoint." << std::endl;
                continue;
            }
        }

        if (!waypoint_accepted) {
            std::cout << "No valid IK solution for this vertical waypoint." << std::endl;
            return false;
        }

        if (contact_found) {
            accumulated_penetration += eef_step;

            if (accumulated_penetration >= penetration) {
                std::cout << "Stopping trajectory after desired contact." << std::endl;
                return true;
            }

            return true;
        }
    }

    std::cout << "Reached search z without detecting desired contact." << std::endl;
    return false;
}

geometry_msgs::msg::Pose eigenToPose(const Eigen::Affine3d &pose_eigen){
    geometry_msgs::msg::Pose pose_msg;

    pose_msg.position.x = pose_eigen.translation().x();
    pose_msg.position.y = pose_eigen.translation().y();
    pose_msg.position.z = pose_eigen.translation().z();

    Eigen::Quaterniond q(pose_eigen.linear());

    pose_msg.orientation.x = q.x();
    pose_msg.orientation.y = q.y();
    pose_msg.orientation.z = q.z();
    pose_msg.orientation.w = q.w();

    return pose_msg;
}

void command_gripper(const rclcpp::Node::SharedPtr& node,
                     double position,
                     double duration_seconds){
    auto gripper_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>("/gripper_position_controller/commands",10);

    rclcpp::sleep_for(std::chrono::milliseconds(500));

    std_msgs::msg::Float64MultiArray msg;

    msg.data = {position, position};

    for (int i = 0; i < 5; ++i){
        gripper_pub->publish(msg);
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(duration_seconds * 1000.0)));
}


bool interpolation_trajectory(std::vector<double> ik_solution, 
							  std::vector<double> stato_iniziale,
							  int num_interpolation,
							  trajectory_msgs::msg::JointTrajectory &trajectory,
							  moveit_planning& robot_planning){

	int count_valid_interpolation;
	bool find = false;

	std::vector<std::pair<std::string, std::string>> contact_pairs;

	double alpha = 1.0 / num_interpolation;

	std::vector<double> stato_finale =ik_solution;  
	count_valid_interpolation=0;

	//stato iniziale e stato finale validi??
	if (robot_planning.check_state_validity(stato_iniziale,contact_pairs) && robot_planning.check_state_validity(stato_finale,contact_pairs)) {

		std::vector<std::vector<double>> correct_state_interpolate;
		
		for (int l = 1; l <= num_interpolation; l++) {
			
			std::vector<double> stato_interpolato;
			for (int j = 0; j < stato_iniziale.size(); j++) {
				double valore_interpolato = (1 - alpha * l) * stato_iniziale[j] + (alpha * l) * stato_finale[j];
				stato_interpolato.push_back(valore_interpolato);
			}
			Eigen::Affine3d posa_punto_interpolato; 
		
			tf2::Quaternion q_punto_interpolato;
			Eigen::Quaterniond qEigen_punto_interpolato;
			q_punto_interpolato.setRPY(stato_interpolato[3], stato_interpolato[4], stato_interpolato[5]);
			qEigen_punto_interpolato.x() = q_punto_interpolato.x();
			qEigen_punto_interpolato.y() = q_punto_interpolato.y();
			qEigen_punto_interpolato.z() = q_punto_interpolato.z();
			qEigen_punto_interpolato.w() = q_punto_interpolato.w();
			posa_punto_interpolato.translation().x() = stato_interpolato[0];
			posa_punto_interpolato.translation().y() = stato_interpolato[1];
			posa_punto_interpolato.translation().z() = stato_interpolato[2];
			posa_punto_interpolato.linear() = qEigen_punto_interpolato.toRotationMatrix();

			robot_planning.forward_kinematics(stato_interpolato, posa_punto_interpolato, "wrist_3_link");
			
			Eigen::Matrix3d rotation_matrix = posa_punto_interpolato.linear();
			Eigen::Vector3d x_axis = rotation_matrix.col(0); // Asse X della matrice di rotazione
			Eigen::Vector3d y_axis = rotation_matrix.col(1); // Asse Y della matrice di rotazione

			// Verifico la validità del punto interpolato
			if (robot_planning.check_state_validity(stato_interpolato, contact_pairs)) {
                count_valid_interpolation++;
                correct_state_interpolate.push_back(stato_interpolato);
            } else {
                std::cout << "Invalid interpolated state at step "
                        << l << "/" << num_interpolation << std::endl;

                std::cout << "State: ";
                for (double q : stato_interpolato) {
                    std::cout << q << " ";
                }
                std::cout << std::endl;

                std::cout << "Contact pairs:" << std::endl;
                for (const auto &pair : contact_pairs) {
                    std::cout << "  " << pair.first << " -- " << pair.second << std::endl;
                }

                return false;
            }
		}

        if (count_valid_interpolation == num_interpolation){
            trajectory.joint_names = {
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint"
            };

            trajectory.points.clear();

            trajectory_msgs::msg::JointTrajectoryPoint start_point;
            start_point.positions = stato_iniziale;
            start_point.time_from_start = rclcpp::Duration::from_seconds(0.0);
            trajectory.points.push_back(start_point);

            for (size_t i = 0; i < correct_state_interpolate.size(); ++i){
                trajectory_msgs::msg::JointTrajectoryPoint point;

                point.positions = correct_state_interpolate[i];

                double duration_seconds = 0.5 * static_cast<double>(i + 1);
                point.time_from_start = rclcpp::Duration::from_seconds(duration_seconds);

                trajectory.points.push_back(point);
            }

            return true;
        }
	}
	return false;
}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);

    auto const node = rclcpp::Node::make_shared("test_pick");
    auto const move_group_node = std::make_shared<rclcpp::Node>("move_group_node");

    rclcpp::executors::SingleThreadedExecutor executor;
	executor.add_node(move_group_node);
	std::thread([&executor]()
				{ executor.spin(); })
		.detach();

	moveit_planning robot_planning =(move_group_node);

    std::string planning_group = node->declare_parameter<std::string>("planning_group","ur_manipulator");

    //******** CONFIG OGGETTO
    std::string object_name = node->declare_parameter<std::string>("object_name","object_box");
    std::vector<double> object_size = node->declare_parameter<std::vector<double>>("object_size",std::vector<double>{0.040, 0.040, 0.100});
    std::vector<double> object_position = node->declare_parameter<std::vector<double>>("object_position",std::vector<double>{-0.500, 0.150, -0.032});
    
    std::vector<double> on_object_position = node->declare_parameter<std::vector<double>>("on_object_position",std::vector<double>{-0.500, 0.150, 0.218});
    std::vector<double> on_object_rpy = node->declare_parameter<std::vector<double>>("on_object_rpy",std::vector<double>{3.14, 0.0, 0.0});

    //******** CONFIG BASKET
    std::vector<double> basket_pos = node->declare_parameter<std::vector<double>>("basket_pos",std::vector<double>{0.387, 0.000, -0.262});
    std::vector<double> basket_size = node->declare_parameter<std::vector<double>>("basket_size",std::vector<double>{0.500, 0.540, 0.310, 0.010});

    //******** GRIPPER
    double gripper_open_position = node->declare_parameter<double>("gripper_open_position", 0.030);
    double gripper_closed_position = node->declare_parameter<double>("gripper_closed_position", 0.026);
    double gripper_motion_duration =node->declare_parameter<double>("gripper_motion_duration", 1.0);
    double target_force = node->declare_parameter<double>("target_force", 40.0);
    double force_threshold = node->declare_parameter<double>("force_threshold", 30.0);
    std::string gripper_link = node->declare_parameter<std::string>("gripper_link", "gripper_base_link");
    
    //******** PARAMS USEFUL
    int num_interpolations = node->declare_parameter<int>("num_interpolations",20);
    double offset_obj_basket = node->declare_parameter<double>("distance_obj_basket",0.080);
    double wall_bin_margin = node->declare_parameter<double>("wall_margin",0.08);
    std::string place_contact_object = node->declare_parameter<std::string>("place_contact_object","basket_bottom");
    std::vector<double> cabinet_place_pos = node->declare_parameter<std::vector<double>>("cabinet_place_pos",std::vector<double>{0.247, 0.000,-0.547});
    std::vector<double> cabinet_place_size = node->declare_parameter<std::vector<double>>("cabinet_place_size",std::vector<double>{0.780, 0.560, 0.285});
    bool use_bin = node->declare_parameter<bool>("use_bin", true);
    bool hard_scene = node->declare_parameter<bool>("hard_scene", true);

    //******** CONFIG ROBOT INIZIALE 
	std::vector<double> robot_config_start = node->declare_parameter<std::vector<double>>("start_config", std::vector<double>{0,-1.57,0.0,-1.57,0.0,0.0});
    Eigen::Affine3d posa_robot_start; 
    robot_planning.set_robot_state(robot_config_start);

    //******** CONFIG ROBOT SOPRA AL TAVOLO  
	std::vector<double> robot_config_on_table = node->declare_parameter<std::vector<double>>("config_on_table", std::vector<double>{0.453786, -1.09956, -1.65806, -1.95477, 1.5708, 0.418879});
    Eigen::Affine3d posa_robot_on_table; 
    robot_planning.forward_kinematics(robot_config_on_table, posa_robot_on_table, "wrist_3_link");
    
    //******** CONFIG ROBOT SOPRA AL CABINET 
	std::vector<double> robot_config_on_cabinet = node->declare_parameter<std::vector<double>>("config_on_cabinet", std::vector<double>{-0.715585, -2.37365, 1.74533, -0.942478, 4.72984, -0.680678});
    Eigen::Affine3d posa_robot_on_cabinet; 
    robot_planning.forward_kinematics(robot_config_on_cabinet, posa_robot_on_cabinet, "wrist_3_link");


    // start
    //********************************************************************************
    // MOVIMENTO DALLA CONFIGURAZIONE INIZIALE A QUELLA SOPRA AL TAVOLO
    //********************************************************************************
    std::cout << "SOLUZIONI INTERPOLAZIONE CONFIG INIZIALE -> CONFIG SOPRA TAVOLO: " << std::endl;

    trajectory_msgs::msg::JointTrajectory traj_start_to_on_table;

    if (interpolation_trajectory(robot_config_on_table, robot_config_start, num_interpolations, traj_start_to_on_table, robot_planning)){
        robot_planning.execute_trajectory(traj_start_to_on_table);
        rclcpp::sleep_for(std::chrono::seconds(5));
    }

    //********************************************************************************
    // MOVIMENTO DALLA CONFIGURAZIONE SOPRA AL TAVOLO A QUELLA SOPRA ALL'OGGETTO DA PRELEVARE
    //********************************************************************************

    Eigen::Affine3d robot_pick_pose;
	tf2::Quaternion q_robot_pick_pose;
	Eigen::Quaterniond qEigen_robot_pick_pose;
	q_robot_pick_pose.setRPY(on_object_rpy[0], on_object_rpy[1], on_object_rpy[2]);
	qEigen_robot_pick_pose.x() = q_robot_pick_pose.x();
	qEigen_robot_pick_pose.y() = q_robot_pick_pose.y();
	qEigen_robot_pick_pose.z() = q_robot_pick_pose.z();
	qEigen_robot_pick_pose.w() = q_robot_pick_pose.w();
	robot_pick_pose.translation().x() = on_object_position[0];
	robot_pick_pose.translation().y() = on_object_position[1];
	robot_pick_pose.translation().z() = on_object_position[2];
	robot_pick_pose.linear() = qEigen_robot_pick_pose.toRotationMatrix();

    std::vector<std::vector<double>> ik_solutions_table_object;

    std::cout << "SOLUZIONI INVERSE KINEMATICHE CONFIG SOPRA TAVOLO -> CONFIG SOPRA OGGETTO: " << std::endl;

	robot_planning.inverse_kinematics(robot_pick_pose, ik_solutions_table_object,robot_config_on_table); 

    trajectory_msgs::msg::JointTrajectory traj_table_to_on_object;
    std::vector<double> on_object_config;
    bool reached_object = false;

    for(int i=0; i<ik_solutions_table_object.size();i++){
        if (interpolation_trajectory(ik_solutions_table_object[i], robot_config_on_table, num_interpolations, traj_table_to_on_object, robot_planning)){
            on_object_config = ik_solutions_table_object[i];
            bool ok = robot_planning.execute_trajectory(traj_table_to_on_object);

            if (!ok) {
                RCLCPP_ERROR(node->get_logger(), "Execution to object failed. Trying another IK solution.");
                continue;
            }

            reached_object = true;
            rclcpp::sleep_for(std::chrono::seconds(5));
            break;
        }
    }

    if (!reached_object) {
        RCLCPP_ERROR(node->get_logger(), "Could not reach object pick pose. Aborting before grasp.");
        rclcpp::shutdown();
        return 1;
    }

    //********************************************************************************
    // GRASPING DELL'OGGETTO
    //********************************************************************************

    RCLCPP_INFO(node->get_logger(), "Closing gripper...");

    command_gripper(node,gripper_closed_position,gripper_motion_duration);

    RCLCPP_INFO(node->get_logger(), "Gripper closed.");

    RCLCPP_INFO(node->get_logger(),"Attaching object '%s' to link '%s'...",object_name.c_str(),gripper_link.c_str());

    robot_planning.attach_object(object_name, gripper_link);

    rclcpp::sleep_for(std::chrono::milliseconds(500));

    robot_planning.print_attached_objects();

    RCLCPP_INFO(node->get_logger(), "Object attached.");


    //********************************************************************************
    // MOVIMENTO DALLA CONFIGURAZIONE SOPRA ALL'OGGETTO (GRASPATO) A QUELLA SOPRA AL TAVOLO 
    //********************************************************************************
    
    trajectory_msgs::msg::JointTrajectory reverse_traj_back_to_table;

    if(interpolation_trajectory(robot_config_on_table,on_object_config, num_interpolations, reverse_traj_back_to_table, robot_planning)){
        robot_planning.execute_trajectory(reverse_traj_back_to_table);
    }

    //********************************************************************************
    // MOVIMENTO DALLA CONFIGURAZIONE SOPRA AL TAVOLO A QUELLA SOPRA AL CABINET
    //********************************************************************************

    std::cout << "SOLUZIONI INVERSE KINEMATICHE CONFIG SOPRA AL TAVOLO -> CONFIG SOPRA AL CABINET: " << std::endl;
    std::vector<std::vector<double>> ik_solutions_table_cabinet;
	robot_planning.inverse_kinematics(posa_robot_on_cabinet, ik_solutions_table_cabinet,robot_config_on_table); 

    std::vector<double> final_on_cabinet_config;
    trajectory_msgs::msg::JointTrajectory traj_table_to_on_cabinet;
    bool reached_cabinet = false;

    for(int i=0; i<ik_solutions_table_cabinet.size();i++){
        if (interpolation_trajectory(ik_solutions_table_cabinet[i], robot_config_on_table, num_interpolations, traj_table_to_on_cabinet, robot_planning)){
            
            bool ok = robot_planning.execute_trajectory(traj_table_to_on_cabinet);

            if (!ok){
                RCLCPP_ERROR(node->get_logger(), "Execution to cabinet failed. Trying another IK solution.");
                continue;
            }

            final_on_cabinet_config = ik_solutions_table_cabinet[i];
            reached_cabinet = true;
            rclcpp::sleep_for(std::chrono::seconds(5));
            break;
        }
    }

    if (!reached_cabinet) {
        RCLCPP_ERROR(node->get_logger(), "Could not reach cabinet pose.");
        rclcpp::shutdown();
        return 1;
    }

    //********************************************************************************
    // MOVIMENTO DALLA CONFIGURAZIONE DA SOPRA AL CABINET A DENTRO ALLA CESTA
    //********************************************************************************

    std::cout << "SOLUZIONI INVERSE KINEMATICS CONFIG SOPRA AL CABINET -> CONFIG DENTRO ALLA CESTA: " << std::endl;
    // Random drop position around the nominal cabinet pose
    std::srand(std::time(nullptr));

    double ee_z_at_pick = on_object_position[2];

    // Offset verticale tra EE e centro oggetto durante il grasp.
    double ee_object_offset_z = ee_z_at_pick - object_position[2];

    std::vector<double> place_area_pos;
    std::vector<double> place_area_size;
    double place_surface_z = 0.0;

    if (use_bin) {
        place_area_pos = basket_pos;
        place_area_size = basket_size;
        place_surface_z = basket_pos[2] + basket_size[3] / 2.0;
        RCLCPP_INFO(node->get_logger(), "Place mode: BIN. Contact object: %s", place_contact_object.c_str());
    
    } else {
        place_area_pos = cabinet_place_pos;
        place_area_size = std::vector<double>{cabinet_place_size[0], cabinet_place_size[1], cabinet_place_size[2], 0.0};
        place_surface_z = cabinet_place_pos[2] + cabinet_place_size[2] - 0.090; 

        RCLCPP_INFO(node->get_logger(), "Place mode: NO BIN. Contact object: %s", place_contact_object.c_str());
    }

    // Quota desiderata del centro dell'oggetto durante il place
    double object_center_pre_place_z = place_surface_z + object_size[2] / 2.0 + offset_obj_basket;

    // Quota desiderata dell'EE durante il place
    double target_ee_pre_place_z = object_center_pre_place_z + ee_object_offset_z;

    // Quota teorica di contatto: fondo oggetto alla quota della superficie del basket.
    double object_contact_target_z = place_surface_z + object_size[2] / 2.0;
    double target_ee_contact_z = object_contact_target_z + ee_object_offset_z;

    std::cout << "EE-object vertical offset: " << ee_object_offset_z << std::endl;
    std::cout << "Place surface z: " << place_surface_z << std::endl;
    std::cout << "Pre-place EE z: " << target_ee_pre_place_z << std::endl;
    std::cout << "Theoretical contact EE z: " << target_ee_contact_z << std::endl;

    // Limiti dentro la cesta
    double min_x = place_area_pos[0] - place_area_size[0] / 2.0 + wall_bin_margin;
    double max_x = place_area_pos[0] + place_area_size[0] / 2.0 - wall_bin_margin;

    double min_y = place_area_pos[1] - place_area_size[1] / 2.0 + wall_bin_margin;
    double max_y = place_area_pos[1] + place_area_size[1] / 2.0 - wall_bin_margin;

    std::cout << "Random x range: [" << min_x << ", " << max_x << "]" << std::endl;
    std::cout << "Random y range: [" << min_y << ", " << max_y << "]" << std::endl;

    trajectory_msgs::msg::JointTrajectory traj_cabinet_to_random_drop;
    trajectory_msgs::msg::JointTrajectory traj_down_to_contact;
    std::vector<double> random_drop_config;
    Eigen::Affine3d random_drop_pose;

    bool reached_valid_place = false;

    int max_attempts = 300;

    for (int i = 0; i < max_attempts && !reached_valid_place; i++) {

        double random_x = min_x + static_cast<double>(std::rand()) / RAND_MAX * (max_x - min_x);
        double random_y = min_y + static_cast<double>(std::rand()) / RAND_MAX * (max_y - min_y);

        random_drop_pose = posa_robot_on_cabinet;

        random_drop_pose.translation().x() = random_x;
        random_drop_pose.translation().y() = random_y;
        random_drop_pose.translation().z() = target_ee_pre_place_z;

        // Mantengo lo stesso orientamento usato sopra al cabinet.
        // In questo modo l'oggetto non viene capovolto.
        random_drop_pose.linear() = posa_robot_on_cabinet.linear();

        std::cout << "Attempt " << i + 1 << " random drop target x: " << random_x << " y: " << random_y << " z: " << target_ee_pre_place_z << std::endl;

        std::vector<std::vector<double>> ik_solutions_cabinet_to_random_drop;
        robot_planning.inverse_kinematics(random_drop_pose,ik_solutions_cabinet_to_random_drop,final_on_cabinet_config);

        if (ik_solutions_cabinet_to_random_drop.empty()) {
            std::cout << "No IK solution for this random pose. Trying another one..." << std::endl;
            continue;
        }

        for (size_t j = 0; j < ik_solutions_cabinet_to_random_drop.size(); j++) {

            trajectory_msgs::msg::JointTrajectory candidate_traj_to_random_drop;

            bool valid_interp = interpolation_trajectory(ik_solutions_cabinet_to_random_drop[j],final_on_cabinet_config,num_interpolations,candidate_traj_to_random_drop,robot_planning);

            if (!valid_interp) {
                std::cout << "IK found, but cabinet -> random_drop interpolation is not valid. Trying another solution/pose..." << std::endl;
                continue;
            }

            random_drop_config = ik_solutions_cabinet_to_random_drop[j];

            std::cout << "Candidate cabinet -> random_drop trajectory is valid. Now checking vertical descent before execution..." << std::endl;

            //********************************************************************************
            // VERIFICA DELLA DISCESA VERTICALE SENZA ANCORA ESEGUIRE 
            //********************************************************************************

            double search_extra_depth = 0.050;
            double target_ee_search_z = target_ee_contact_z - search_extra_depth;

            trajectory_msgs::msg::JointTrajectory candidate_traj_down_to_contact;
            double eef_step = 0.003;
            double penetration = 0.003;

            std::cout << "Vertical descent search from z " << random_drop_pose.translation().z() << " to z " << target_ee_search_z << " using contact object: " << place_contact_object<< std::endl;

            bool vertical_ok = plan_vertical_cartesian_until_contact_from_state(random_drop_pose,random_drop_config,target_ee_search_z,place_contact_object,object_name,eef_step,penetration,candidate_traj_down_to_contact,robot_planning);

            if (!vertical_ok) {
                std::cout << "Vertical descent not valid from this random_drop_config. Trying another pose..." << std::endl;
                continue;
            }

            std::cout << "Both trajectories are valid. Executing cabinet -> random_drop..." << std::endl;

            bool ok_pre_place = robot_planning.execute_trajectory(candidate_traj_to_random_drop);

            if (!ok_pre_place) {
                RCLCPP_ERROR(node->get_logger(), "Execution to random drop pose failed. Trying another pose.");
                continue;
            }

            rclcpp::sleep_for(std::chrono::seconds(1));

            std::cout << "Executing vertical descent to contact..." << std::endl;

            bool exec_down_ok = robot_planning.execute_trajectory(candidate_traj_down_to_contact);

            if (!exec_down_ok) {
                RCLCPP_WARN(node->get_logger(), "Vertical descent execution failed. Trying another pose.");
                continue;
            }

            traj_cabinet_to_random_drop = candidate_traj_to_random_drop;
            traj_down_to_contact = candidate_traj_down_to_contact;
            reached_valid_place = true;

            std::cout << "Valid pre-place and vertical descent completed. Ready to release object." << std::endl;
            rclcpp::sleep_for(std::chrono::seconds(2));
            break;
        }
    }

    if (!reached_valid_place) {
        RCLCPP_ERROR(node->get_logger(), "Could not find any random pose valid for both cabinet trajectory and vertical descent.");
        rclcpp::shutdown();
        return 1;
    }

    //********************************************************************************
    // RILASCIO DELL'OGGETTO
    //********************************************************************************
    RCLCPP_INFO(node->get_logger(), "Opening gripper to release object...");

    command_gripper(node, gripper_open_position, gripper_motion_duration);
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    robot_planning.detach_object(object_name);
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    RCLCPP_INFO(node->get_logger(), "Object detached and released.");

    //********************************************************************************
    // RISALITA VERTICALE E RITORNO SOPRA AL CABINET
    //********************************************************************************
    if (!traj_down_to_contact.points.empty()) {
        trajectory_msgs::msg::JointTrajectory traj_up_from_contact;
        traj_up_from_contact.joint_names = traj_down_to_contact.joint_names;

        double dt = 0.25;
        int out_index = 0;

        for (auto it = traj_down_to_contact.points.rbegin(); it != traj_down_to_contact.points.rend(); ++it) {
            trajectory_msgs::msg::JointTrajectoryPoint p = *it;
            p.time_from_start = rclcpp::Duration::from_seconds(dt * static_cast<double>(out_index));
            traj_up_from_contact.points.push_back(p);
            out_index++;
        }

        std::cout << "Executing vertical ascent after release..." << std::endl;
        robot_planning.execute_trajectory(traj_up_from_contact);
        rclcpp::sleep_for(std::chrono::seconds(2));
    }

    trajectory_msgs::msg::JointTrajectory traj_back_to_cabinet;
    bool back_ok = interpolation_trajectory(final_on_cabinet_config,random_drop_config,num_interpolations,traj_back_to_cabinet,robot_planning);

    if (back_ok) {
        std::cout << "Returning above cabinet..." << std::endl;
        robot_planning.execute_trajectory(traj_back_to_cabinet);
        rclcpp::sleep_for(std::chrono::seconds(2));
    } else {
        RCLCPP_ERROR(node->get_logger(), "Could not return to cabinet after release.");
    }

    rclcpp::shutdown();
    return 0;
}