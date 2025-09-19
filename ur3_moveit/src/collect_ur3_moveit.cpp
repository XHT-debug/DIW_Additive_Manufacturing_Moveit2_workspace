#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>


#include <rclcpp/rclcpp.hpp>

#include <iostream>
#include <fstream>
#include <sstream>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("node name: ur3_moveit");
double offset_x, offset_y, offset_z;

moveit_msgs::msg::RobotTrajectory scale_trajectory_speed(moveit_msgs::msg::RobotTrajectory trajectory, double scale)
{
	// moveit_msgs::msg::RobotTrajectory new_trajectory;
	// new_trajectory.joint_trajectory = trajectory.joint_trajectory;

	int n_joints = trajectory.joint_trajectory.joint_names.size();

	int n_points = trajectory.joint_trajectory.points.size();

	for (int i = 0; i < n_points; i++)
	{
		
		double seconds = trajectory.joint_trajectory.points[i].time_from_start.sec + 
        				 trajectory.joint_trajectory.points[i].time_from_start.nanosec / 1e9;
		seconds *= 1 / scale; 
		int sec = static_cast<int>(seconds); // 秒数
		uint32_t nanosec = static_cast<uint32_t>((seconds - sec) * 1e9);
		builtin_interfaces::msg::Duration duration_msg;
		duration_msg.sec = sec;
		duration_msg.nanosec = nanosec;
		trajectory.joint_trajectory.points[i].time_from_start = duration_msg;


		for (int j = 0; j < n_joints; j++)
		{
			trajectory.joint_trajectory.points[i].velocities[j] *= scale;
			trajectory.joint_trajectory.points[i].accelerations[j] *= scale*scale;
		}
	}

	return trajectory;
	
}


int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
  	rclcpp::NodeOptions node_options;
	node_options.automatically_declare_parameters_from_overrides(true);
	auto move_group_node = rclcpp::Node::make_shared("ur3_moveit", node_options);

	rclcpp::executors::SingleThreadedExecutor executor;
	executor.add_node(move_group_node);
	std::thread([&executor]() { executor.spin(); }).detach();

	static const std::string PLANNING_GROUP = "ur_manipulator";

	auto velocity_collect_pub = move_group_node->create_publisher<std_msgs::msg::Int32>("/velocity_collect_switch", 10);
	

	moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;



	// const moveit::core::JointModelGroup* joint_model_group =
	// 	move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
	namespace rvt = rviz_visual_tools;
	moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "world", "move_group_tutorial",
														move_group.getRobotModel());

	visual_tools.deleteAllMarkers();
	visual_tools.loadRemoteControl();
	Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
	text_pose.translation().z() = 1.0;
	visual_tools.publishText(text_pose, "MoveGroupInterface_Demo", rvt::WHITE, rvt::XLARGE);
	visual_tools.trigger();

	RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
	RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());
	RCLCPP_INFO(LOGGER, "Available Planning Groups:");
	std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
				std::ostream_iterator<std::string>(std::cout, ", "));

	 // Start the demo
	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to initialize the workspace");

	double tcpTl6_x,tcpTl6_y,tcpTl6_z, tcpTl6_qx,tcpTl6_qy,tcpTl6_qz,tcpTl6_qw;

	move_group_node->get_parameter("tcpTl6_x",tcpTl6_x);
	move_group_node->get_parameter("tcpTl6_y",tcpTl6_y);
	move_group_node->get_parameter("tcpTl6_z",tcpTl6_z);
	move_group_node->get_parameter("tcpTl6_qx",tcpTl6_qx);
	move_group_node->get_parameter("tcpTl6_qy",tcpTl6_qy);
	move_group_node->get_parameter("tcpTl6_qz",tcpTl6_qz);
	move_group_node->get_parameter("tcpTl6_qw",tcpTl6_qw);


	auto tf_buffer = std::make_shared<tf2_ros::Buffer>(move_group_node->get_clock());
  	tf2_ros::TransformListener tf_listener(*tf_buffer);

	geometry_msgs::msg::TransformStamped transformstamped_bTs;
	transformstamped_bTs = tf_buffer->lookupTransform("world", "ft_frame", tf2::TimePointZero, tf2::durationFromSec(10.0));
	tf2::Vector3 translation_bTs(transformstamped_bTs.transform.translation.x,transformstamped_bTs.transform.translation.y,transformstamped_bTs.transform.translation.z);
	tf2::Quaternion quaternion_bTs(transformstamped_bTs.transform.rotation.x,transformstamped_bTs.transform.rotation.y,transformstamped_bTs.transform.rotation.z,transformstamped_bTs.transform.rotation.w);
	tf2::Transform transform_bTs(quaternion_bTs, translation_bTs);

	tf2::Vector3 translation_tcpTl6(tcpTl6_x,tcpTl6_y,tcpTl6_z);
	tf2::Quaternion quaternion_tcpTl6(tcpTl6_qx,tcpTl6_qy,tcpTl6_qz,tcpTl6_qw);
	tf2::Transform transform_tcpTl6(quaternion_tcpTl6, translation_tcpTl6);


	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

	// 回归原点，准备开始打印

	geometry_msgs::msg::Pose target_pose, visual_pose;
	std::vector<geometry_msgs::msg::Pose> waypoints_first;

	target_pose.position.x = -0.365;
	offset_x = 0.005;
	target_pose.position.y = 0.050;
	offset_y = 0;
	target_pose.position.z = 0.275;
	offset_z = 0.00568;
	target_pose.orientation.x = 1.0;
	target_pose.orientation.y = 0.0;
	target_pose.orientation.z = 0.0;
	target_pose.orientation.w = 0.0;

	waypoints_first.push_back(target_pose);

	moveit_msgs::msg::RobotTrajectory trajectory_first;
	const double jump_threshold = 0.0;
	const double eef_step = 0.01;
	double fraction_first = move_group.computeCartesianPath(waypoints_first, eef_step, jump_threshold, trajectory_first);

	moveit_msgs::msg::RobotTrajectory new_trajectory_first;
	new_trajectory_first = scale_trajectory_speed(trajectory_first, 0.1);
	move_group.setMaxVelocityScalingFactor(0.005);
	move_group.setMaxAccelerationScalingFactor(0.005);
	move_group.execute(new_trajectory_first);
	// 第一段返回打印原点结束

	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

	// 添加从txt文件中提取的轨迹点 
  	std::vector<geometry_msgs::msg::Pose> waypoints, visual_waypoints;

	std::string planning_filename;
	move_group_node->get_parameter("planning_filename", planning_filename);
	bool including_pose_estimation;
	move_group_node->get_parameter("including_pose_estimation", including_pose_estimation);
	std::string planning_filepath = "/home/xht/moveit2_workspace/src/ur3_moveit/config/plan/test.txt"; 
	RCLCPP_INFO(LOGGER, "planning_filepath %s", planning_filepath.c_str());
	std::ifstream inputFile(planning_filepath); 

	if (!inputFile.is_open()) {
		RCLCPP_INFO(LOGGER, "Fail to load file");
		return 1;
	}
	std::string line;
	while (std::getline(inputFile, line)) {
		std::istringstream iss(line);
		tf2::Quaternion q;
		double x, y, z,q_x,q_y,q_z, q_w;
		char dummy;
		iss >> dummy >> x;
		iss >> dummy >> y;
		iss >> dummy >> z;
		iss >> dummy >> q_x;
		iss >> dummy >> q_y;
		iss >> dummy >> q_z;
		iss >> dummy >> q_w;
		tf2::Vector3 translation_sTtcp(x, y, z);
		tf2::Quaternion quaternion_sTtcp;
		if (including_pose_estimation){
			quaternion_sTtcp = tf2::Quaternion(q_x, q_y, q_z, q_w); // with pose estimation
		} 
		else{
			quaternion_sTtcp=quaternion_bTs.inverse(); // without pose estimation
		}

		target_pose.position.x = x + offset_x;
		target_pose.position.y = y + offset_y;
		target_pose.position.z = z + offset_z;
		target_pose.orientation.x = q_x;
		target_pose.orientation.y = q_y;
		target_pose.orientation.z = q_z;
		target_pose.orientation.w = q_w;

		waypoints.push_back(target_pose);
	}
	inputFile.close();

	
	moveit_msgs::msg::RobotTrajectory trajectory;

	double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
	RCLCPP_INFO(LOGGER, "Visualizing plan Cartesian path (%.2f%% achieved)", fraction * 100.0);

	// 笛卡尔路径的速度规划
	
	// moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);
	// std::vector<double> joint_group_positions;
	// current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
	moveit_msgs::msg::RobotTrajectory new_trajectory;
	new_trajectory = scale_trajectory_speed(trajectory, 0.024);
	// moveit::planning_interface::MoveItErrorCode success = move_group.plan(plan);
	moveit::planning_interface::MoveGroupInterface::Plan plan;
	plan.trajectory_ = new_trajectory;


	visual_tools.deleteAllMarkers();
	visual_tools.publishText(text_pose, "Cartesian_Path", rvt::WHITE, rvt::XLARGE);
	visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
	// for (std::size_t i = 0; i < waypoints.size(); ++i)
  	// 	visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
	visual_tools.publishPath(visual_waypoints, rvt::RED, rvt::SMALL);
	visual_tools.trigger();

	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to fabricate the sensor");
	// API_set_laser_on()


	auto start_time = std::chrono::high_resolution_clock::now();
	// move_group.execute(new_trajectory); // execute the plan
    std_msgs::msg::Int32 velocity_collect_control;

    velocity_collect_control.data = 1;
	velocity_collect_pub->publish(velocity_collect_control);

	move_group.execute(plan);

    velocity_collect_control.data = 0;
	velocity_collect_pub->publish(velocity_collect_control);

	auto end_time = std::chrono::high_resolution_clock::now();

    // 计算执行时间
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

    // 输出执行时间
    std::cout << "Trajectory execution time: " << duration << " ms" << std::endl;



	// API_set_laser_on()
	visual_tools.deleteAllMarkers();
	visual_tools.trigger();
	rclcpp::shutdown();
	return 0;
}
