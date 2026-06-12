#include <rclcpp/rclcpp.hpp>

#include <moveit_planning/moveit_planning.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <std_msgs/msg/float64_multi_array.hpp>

#include <string>
#include <vector>
#include <chrono>

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
			if (!robot_planning.check_state_validity(stato_interpolato,contact_pairs)) {
				// Punto interpolato non valido
				std::cout << "Interpolazione non valida al punto " << l << std::endl;
				correct_state_interpolate.clear();
				break;                
			}else{
				count_valid_interpolation++;
				correct_state_interpolate.push_back(stato_interpolato);
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
    std::vector<double> object_position = node->declare_parameter<std::vector<double>>("object_position",std::vector<double>{-0.500, -0.100, -0.032});
    
    std::vector<double> on_object_position = node->declare_parameter<std::vector<double>>("on_object_position",std::vector<double>{-0.500, -0.100, 0.218});
    std::vector<double> on_object_rpy = node->declare_parameter<std::vector<double>>("on_object_rpy",std::vector<double>{3.14, 0.0, 0.0});

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


    //******** NUM INTERPOLAZIONI
    int num_interpolations = node->declare_parameter<int>("num_interpolazioni",10);

    //******** GRIPPER
    double gripper_open_position = node->declare_parameter<double>("gripper_open_position", 0.030);
    double gripper_closed_position = node->declare_parameter<double>("gripper_closed_position", 0.026);
    double gripper_motion_duration =node->declare_parameter<double>("gripper_motion_duration", 1.0);
    std::string gripper_link = node->declare_parameter<std::string>("gripper_link", "gripper_base_link");


    // start
    std::vector<std::vector<double>> ik_solutions_start_table;

    std::cout << "SOLUZIONI INVERSE KINEMATICHE CONFIG INIZIALE -> CONFIG SOPRA TAVOLO: " << std::endl;

	robot_planning.inverse_kinematics(posa_robot_on_table, ik_solutions_start_table,robot_config_start); 

    //********************************************************************************
    // MOVIMENTO DALLA CONFIGURAZIONE INIZIALE A QUELLA SOPRA AL TAVOLO
    trajectory_msgs::msg::JointTrajectory traj_start_to_on_table;

    for(int i=0; i<ik_solutions_start_table.size();i++){
        if (interpolation_trajectory(ik_solutions_start_table[i], robot_config_start, num_interpolations, traj_start_to_on_table, robot_planning)){
            robot_planning.execute_trajectory(traj_start_to_on_table);
            rclcpp::sleep_for(std::chrono::seconds(5));
            break;
        }
    }

    //********************************************************************************
    // MOVIMENTO DALLA CONFIGURAZIONE SOPRA AL TAVOLO A QUELLA SOPRA ALL'OGGETTO DA PRELEVARE
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

    for(int i=0; i<ik_solutions_table_object.size();i++){
        if (interpolation_trajectory(ik_solutions_table_object[i], robot_config_on_table, num_interpolations, traj_table_to_on_object, robot_planning)){
            on_object_config = ik_solutions_table_object[i];
            robot_planning.execute_trajectory(traj_table_to_on_object);
            rclcpp::sleep_for(std::chrono::seconds(5));
            break;
        }
    }

    //********************************************************************************
    // GRASPING DELL'OGGETTO
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

    
    trajectory_msgs::msg::JointTrajectory reverse_traj_back_to_table;

    if(interpolation_trajectory(robot_config_on_table,on_object_config, num_interpolations, reverse_traj_back_to_table, robot_planning)){
        robot_planning.execute_trajectory(reverse_traj_back_to_table);
    }

    //********************************************************************************
    // MOVIMENTO DALLA CONFIGURAZIONE SOPRA ALL'OGGETTO (GRASPATO) A QUELLA SOPRA AL TAVOLO 
    std::cout << "SOLUZIONI INVERSE KINEMATICHE CONFIG SOPRA AL TAVOLO -> CONFIG SOPRA AL CABINET: " << std::endl;
    std::vector<std::vector<double>> ik_solutions_table_cabinet;
	robot_planning.inverse_kinematics(posa_robot_on_cabinet, ik_solutions_table_cabinet,robot_config_on_table); 

    trajectory_msgs::msg::JointTrajectory traj_table_to_on_cabinet;

    for(int i=0; i<ik_solutions_table_cabinet.size();i++){
        if (interpolation_trajectory(ik_solutions_table_cabinet[i], robot_config_on_table, num_interpolations, traj_table_to_on_cabinet, robot_planning)){
            robot_planning.execute_trajectory(traj_table_to_on_cabinet);
            rclcpp::sleep_for(std::chrono::seconds(5));
            break;
        }
    }


    //********************************************************************************
    // MOVIMENTO DALLA CONFIGURAZIONE SOPRA ALL'OGGETTO (GRASPATO) A QUELLA SOPRA AL TAVOLO 


    //********************************************************************************
    // MOVIMENTO DALLA CONFIGURAZIONE SOPRA ALL'OGGETTO (GRASPATO) A QUELLA SOPRA AL TAVOLO 

    rclcpp::shutdown();
    return 0;
}