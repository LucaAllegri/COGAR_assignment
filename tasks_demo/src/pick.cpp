#include <rclcpp/rclcpp.hpp>

#include <moveit_planning/moveit_planning.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <std_msgs/msg/float64_multi_array.hpp>

#include <string>
#include <vector>
#include <chrono>


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


    //******** NUM INTERPOLAZIONI
    int num_interpolations = node->declare_parameter<int>("num_interpolazioni",10);

    //******** CONFIG ROBOT INIZIALE 
	std::vector<double> robot_config_start = node->declare_parameter<std::vector<double>>("start_config", std::vector<double>{0,-1.57,0.0,-1.57,0.0,0.0});

    //******** CONFIG ROBOT SOPRA AL TAVOLO  
	std::vector<double> robot_config_on_table = node->declare_parameter<std::vector<double>>("config_on_table", std::vector<double>{0.453786, -1.09956, -1.65806, -1.95477, 1.5708, 0.418879});

    //******** CONFIG ROBOT SOPRA AL CABINET 
	std::vector<double> robot_config_on_cabinet = node->declare_parameter<std::vector<double>>("config_on_cabinet", std::vector<double>{-0.715585, -2.37365, 1.74533, -0.942478, 4.72984, -0.680678});

    trajectory_msgs::msg::JointTrajectory traj_start_to_on_table;
    robot_planning.set_robot_state(robot_config_start);


    if (!interpolation_trajectory(robot_config_on_table,robot_config_start,num_interpolations,traj_start_to_on_table,robot_planning)){
        RCLCPP_ERROR(node->get_logger(),"Failed to plan trajectory from start_config to config_pre_pick.");
        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(node->get_logger(),"Trajectory from start_config to config_pre_pick found.");

    rclcpp::sleep_for(std::chrono::seconds(10));

    robot_planning.execute_trajectory(traj_start_to_on_table);

    rclcpp::sleep_for(std::chrono::seconds(5));


    rclcpp::shutdown();
    return 0;
}