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
#include <chrono>
#include <thread>

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
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("node name: ur3_moveit");
double offset_x, offset_y, offset_z;
double kong_scale = 0.06, go_scale = 0.001;

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
			trajectory.joint_trajectory.points[i].accelerations[j] *= scale * scale;
		}
	}

	return trajectory;
	
}

std::vector<geometry_msgs::msg::Pose> read_to_waypoints(std::string planning_filepath)
{
	std::vector<geometry_msgs::msg::Pose> waypoints;
	geometry_msgs::msg::Pose target_pose;

	RCLCPP_INFO(LOGGER, "planning_filepath %s", planning_filepath.c_str());
	std::ifstream inputFile(planning_filepath); 

	if (!inputFile.is_open()) {
		RCLCPP_INFO(LOGGER, "Fail to load file");
		return waypoints;
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

	return waypoints;
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

	auto pressure_control_pub = move_group_node->create_publisher<std_msgs::msg::Int32>("/pressure_control", 10);
	auto pressure_adjust_pub = move_group_node->create_publisher<std_msgs::msg::Int32>("/pressure_adjust", 10);
	

	moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

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

	// 将气压进口关闭
	std_msgs::msg::Int32 msg_pressure_control, msg_pressure_adjust; 
	msg_pressure_control.data = 1;  
	pressure_control_pub->publish(msg_pressure_control);

	// 调节比例阀，气压为0.6bar
	msg_pressure_adjust.data = 150;  // 示例压力值
	pressure_adjust_pub->publish(msg_pressure_adjust);


	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

	geometry_msgs::msg::Pose target_pose, visual_pose;

	std::vector<geometry_msgs::msg::Pose> waypoints_first, waypoints_qingkong;

	target_pose.position.x = -0.3400; // -0.3546
	target_pose.position.y = 0.050; // 0.079759
	target_pose.position.z = 0.280; // 0.274
	target_pose.orientation.x = 1.0;
	target_pose.orientation.y = 0.0;
	target_pose.orientation.z = 0.0;
	target_pose.orientation.w = 0.0;
	waypoints_qingkong.push_back(target_pose);

	moveit_msgs::msg::RobotTrajectory trajectory_qingkong;
	const double jump_threshold = 0.0;
	const double eef_step = 0.0001; // 0.1mm
	double fraction_first = move_group.computeCartesianPath(waypoints_qingkong, eef_step, jump_threshold, trajectory_qingkong);
	
	moveit_msgs::msg::RobotTrajectory new_trajectory_qingkong;
	new_trajectory_qingkong = scale_trajectory_speed(trajectory_qingkong, 0.1);

	move_group.execute(new_trajectory_qingkong);
	
	// msg_pressure_control.data = 0;  
	// pressure_control_pub->publish(msg_pressure_control);

	// std::this_thread::sleep_for(std::chrono::seconds(2));

	// msg_pressure_control.data = 1;  
	// pressure_control_pub->publish(msg_pressure_control);

	
	
	// 回归原点，准备开始打印
	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

	
	target_pose.position.x = -0.3530; // -0.3546
	offset_x = target_pose.position.x - (-0.3546);
	target_pose.position.y = 0.080199; // 0.079759
	offset_y = 0.079699 - (0.079759);
	target_pose.position.z = 0.271; // 0.274
	offset_z = target_pose.position.z - (0.274);
	target_pose.orientation.x = 1.0;
	target_pose.orientation.y = 0.0;
	target_pose.orientation.z = 0.0;
	target_pose.orientation.w = 0.0;

	waypoints_first.push_back(target_pose);

	moveit_msgs::msg::RobotTrajectory trajectory_first;
	fraction_first = move_group.computeCartesianPath(waypoints_first, eef_step, jump_threshold, trajectory_first);

	moveit_msgs::msg::RobotTrajectory new_trajectory_first;
	new_trajectory_first = scale_trajectory_speed(trajectory_first, 0.1);

	move_group.setMaxVelocityScalingFactor(0.005);
	move_group.setMaxAccelerationScalingFactor(0.005);
	move_group.execute(new_trajectory_first);
	// 第一段返回打印原点结束

	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

	// 添加从txt文件中提取的轨迹点 
	std::vector<geometry_msgs::msg::Pose> waypoints_1_1 = read_to_waypoints("/home/xht/moveit2_workspace/src/ur3_moveit/config/plan/connect_1/electrical_connect_test_1_1.txt");

	moveit_msgs::msg::RobotTrajectory trajectory_1_1;
	double fraction = move_group.computeCartesianPath(waypoints_1_1, eef_step, jump_threshold, trajectory_1_1);
	RCLCPP_INFO(LOGGER, "Visualizing plan Cartesian path (%.2f%% achieved)", fraction * 100.0);
	// 笛卡尔路径的速度规划
	// moveit_msgs::msg::RobotTrajectory trajectory_scaled_1 = scale_trajectory_speed(trajectory_1, 0.01);
	// plan.trajectory_ = trajectory_scaled_1;

	trajectory_processing::TimeOptimalTrajectoryGeneration time_param(0.1, 0.02, 0.001);
	robot_trajectory::RobotTrajectory rt(move_group.getRobotModel(), move_group.getName());
    rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory_1_1);
	time_param.computeTimeStamps(rt, 2.5 * 0.001 * 0.8, 0.001); // 0.02对应25mm/s
	rt.getRobotTrajectoryMsg(trajectory_1_1);
	moveit::planning_interface::MoveGroupInterface::Plan plan;
	plan.trajectory_ = trajectory_1_1;

	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to fabricate the sensor");
	// API_set_laser_on()

	// 将气压进口打开 
	msg_pressure_control.data = 0;  // 示例压力值
	pressure_control_pub->publish(msg_pressure_control);

	auto start_time = std::chrono::high_resolution_clock::now();
	// move_group.execute(new_trajectory); // execute the plan

	move_group.execute(plan);

	auto end_time = std::chrono::high_resolution_clock::now();

    // 计算执行时间
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

    // 输出执行时间
    std::cout << "Trajectory execution time: " << duration << " ms" << std::endl;

	// 将气压进口关闭 
	msg_pressure_control.data = 1;  // 示例压力值
	pressure_control_pub->publish(msg_pressure_control);

	std::vector<geometry_msgs::msg::Pose> waypoints_1_2 = read_to_waypoints("/home/xht/moveit2_workspace/src/ur3_moveit/config/plan/connect_1/electrical_connect_test_1_2.txt");
	moveit_msgs::msg::RobotTrajectory trajectory_1_2;
	fraction = move_group.computeCartesianPath(waypoints_1_2, eef_step, jump_threshold, trajectory_1_2);
	RCLCPP_INFO(LOGGER, "Visualizing plan Cartesian path (%.2f%% achieved)", fraction * 100.0);
	moveit_msgs::msg::RobotTrajectory trajectory_scaled_1_2 = scale_trajectory_speed(trajectory_1_2, kong_scale);
	plan.trajectory_ = trajectory_scaled_1_2;
	move_group.execute(plan);

	// 打印第二段
	
	std::vector<geometry_msgs::msg::Pose> waypoints_2_1 = read_to_waypoints("/home/xht/moveit2_workspace/src/ur3_moveit/config/plan/connect_1/electrical_connect_test_2_1.txt");
	moveit_msgs::msg::RobotTrajectory trajectory_2_1;
	fraction = move_group.computeCartesianPath(waypoints_2_1, eef_step, jump_threshold, trajectory_2_1);
	RCLCPP_INFO(LOGGER, "Visualizing plan Cartesian path (%.2f%% achieved)", fraction * 100.0);
	// 笛卡尔路径的速度规划
	// moveit_msgs::msg::RobotTrajectory trajectory_scaled_1 = scale_trajectory_speed(trajectory_1, 0.01);
	// plan.trajectory_ = trajectory_scaled_1;
    rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory_2_1);
	time_param.computeTimeStamps(rt, 2.5 * 0.001 * 0.8, 0.001); // 0.02对应25mm/s
    rt.getRobotTrajectoryMsg(trajectory_2_1);
	plan.trajectory_ = trajectory_2_1;

	msg_pressure_control.data = 0;  // 示例压力值
	pressure_control_pub->publish(msg_pressure_control);  // 将气压进口打开 

	move_group.execute(plan);

	msg_pressure_control.data = 1;  // 示例压力值
	pressure_control_pub->publish(msg_pressure_control);  // 将气压进口关闭 

	std::vector<geometry_msgs::msg::Pose> waypoints_2_2 = read_to_waypoints("/home/xht/moveit2_workspace/src/ur3_moveit/config/plan/connect_1/electrical_connect_test_2_2.txt");
	moveit_msgs::msg::RobotTrajectory trajectory_2_2;
	fraction = move_group.computeCartesianPath(waypoints_2_2, eef_step, jump_threshold, trajectory_2_2);
	RCLCPP_INFO(LOGGER, "Visualizing plan Cartesian path (%.2f%% achieved)", fraction * 100.0);
	moveit_msgs::msg::RobotTrajectory trajectory_scaled_2_2 = scale_trajectory_speed(trajectory_2_2, kong_scale);
	plan.trajectory_ = trajectory_scaled_2_2;
	move_group.execute(plan);

	// 打印第三段
	
	std::vector<geometry_msgs::msg::Pose> waypoints_3_1 = read_to_waypoints("/home/xht/moveit2_workspace/src/ur3_moveit/config/plan/connect_1/electrical_connect_test_3_1.txt");
	moveit_msgs::msg::RobotTrajectory trajectory_3_1;
	fraction = move_group.computeCartesianPath(waypoints_3_1, eef_step, jump_threshold, trajectory_3_1);
	RCLCPP_INFO(LOGGER, "Visualizing plan Cartesian path (%.2f%% achieved)", fraction * 100.0);
	// 笛卡尔路径的速度规划
	// moveit_msgs::msg::RobotTrajectory trajectory_scaled_1 = scale_trajectory_speed(trajectory_1, 0.01);
	// plan.trajectory_ = trajectory_scaled_1;
    rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory_3_1);
	time_param.computeTimeStamps(rt, 2.5 * 0.001 * 0.8, 0.001); // 0.02对应25mm/s
    rt.getRobotTrajectoryMsg(trajectory_3_1);
	plan.trajectory_ = trajectory_3_1;

	msg_pressure_control.data = 0;  // 示例压力值
	pressure_control_pub->publish(msg_pressure_control);  // 将气压进口打开 

	move_group.execute(plan);

	msg_pressure_control.data = 1;  // 示例压力值
	pressure_control_pub->publish(msg_pressure_control);  // 将气压进口关闭 

	std::vector<geometry_msgs::msg::Pose> waypoints_3_2 = read_to_waypoints("/home/xht/moveit2_workspace/src/ur3_moveit/config/plan/connect_1/electrical_connect_test_3_2.txt");
	moveit_msgs::msg::RobotTrajectory trajectory_3_2;
	fraction = move_group.computeCartesianPath(waypoints_3_2, eef_step, jump_threshold, trajectory_3_2);
	RCLCPP_INFO(LOGGER, "Visualizing plan Cartesian path (%.2f%% achieved)", fraction * 100.0);
	moveit_msgs::msg::RobotTrajectory trajectory_scaled_3_2 = scale_trajectory_speed(trajectory_3_2, kong_scale);
	plan.trajectory_ = trajectory_scaled_3_2;
	move_group.execute(plan);

	// 打印第四段
	
	std::vector<geometry_msgs::msg::Pose> waypoints_4_1 = read_to_waypoints("/home/xht/moveit2_workspace/src/ur3_moveit/config/plan/connect_1/electrical_connect_test_4_1.txt");
	moveit_msgs::msg::RobotTrajectory trajectory_4_1;
	fraction = move_group.computeCartesianPath(waypoints_4_1, eef_step, jump_threshold, trajectory_4_1);
	RCLCPP_INFO(LOGGER, "Visualizing plan Cartesian path (%.2f%% achieved)", fraction * 100.0);
	// 笛卡尔路径的速度规划
	// moveit_msgs::msg::RobotTrajectory trajectory_scaled_1 = scale_trajectory_speed(trajectory_1, 0.01);
	// plan.trajectory_ = trajectory_scaled_1;
    rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory_4_1);
	time_param.computeTimeStamps(rt, 2.5 * 0.001 * 0.8, 0.001); // 0.02对应25mm/s
    rt.getRobotTrajectoryMsg(trajectory_4_1);
	plan.trajectory_ = trajectory_4_1;

	msg_pressure_control.data = 0;  // 示例压力值
	pressure_control_pub->publish(msg_pressure_control);  // 将气压进口打开 

	move_group.execute(plan);

	msg_pressure_control.data = 1;  // 示例压力值
	pressure_control_pub->publish(msg_pressure_control);  // 将气压进口关闭 

	std::vector<geometry_msgs::msg::Pose> waypoints_4_2 = read_to_waypoints("/home/xht/moveit2_workspace/src/ur3_moveit/config/plan/connect_1/electrical_connect_test_4_2.txt");
	moveit_msgs::msg::RobotTrajectory trajectory_4_2;
	fraction = move_group.computeCartesianPath(waypoints_4_2, eef_step, jump_threshold, trajectory_4_2);
	RCLCPP_INFO(LOGGER, "Visualizing plan Cartesian path (%.2f%% achieved)", fraction * 100.0);
	moveit_msgs::msg::RobotTrajectory trajectory_scaled_4_2 = scale_trajectory_speed(trajectory_4_2, kong_scale);
	plan.trajectory_ = trajectory_scaled_4_2;
	move_group.execute(plan);

	// 打印第五段
	
	std::vector<geometry_msgs::msg::Pose> waypoints_5_1 = read_to_waypoints("/home/xht/moveit2_workspace/src/ur3_moveit/config/plan/connect_1/electrical_connect_test_5_1.txt");
	moveit_msgs::msg::RobotTrajectory trajectory_5_1;
	fraction = move_group.computeCartesianPath(waypoints_5_1, eef_step, jump_threshold, trajectory_5_1);
	RCLCPP_INFO(LOGGER, "Visualizing plan Cartesian path (%.2f%% achieved)", fraction * 100.0);
	// 笛卡尔路径的速度规划
	// moveit_msgs::msg::RobotTrajectory trajectory_scaled_1 = scale_trajectory_speed(trajectory_1, 0.01);
	// plan.trajectory_ = trajectory_scaled_1;
    rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory_5_1);
	time_param.computeTimeStamps(rt, 2.5 * 0.001 * 0.8, 0.001); // 0.02对应25mm/s
    rt.getRobotTrajectoryMsg(trajectory_5_1);
	plan.trajectory_ = trajectory_5_1;

	msg_pressure_control.data = 0;  // 示例压力值
	pressure_control_pub->publish(msg_pressure_control);  // 将气压进口打开 

	move_group.execute(plan);

	msg_pressure_control.data = 1;  // 示例压力值
	pressure_control_pub->publish(msg_pressure_control);  // 将气压进口关闭 

	std::vector<geometry_msgs::msg::Pose> waypoints_5_2 = read_to_waypoints("/home/xht/moveit2_workspace/src/ur3_moveit/config/plan/connect_1/electrical_connect_test_5_2.txt");
	moveit_msgs::msg::RobotTrajectory trajectory_5_2;
	fraction = move_group.computeCartesianPath(waypoints_5_2, eef_step, jump_threshold, trajectory_5_2);
	RCLCPP_INFO(LOGGER, "Visualizing plan Cartesian path (%.2f%% achieved)", fraction * 100.0);
	moveit_msgs::msg::RobotTrajectory trajectory_scaled_5_2 = scale_trajectory_speed(trajectory_5_2, kong_scale);
	plan.trajectory_ = trajectory_scaled_5_2;
	move_group.execute(plan);

	// 打印第六段

	std::vector<geometry_msgs::msg::Pose> waypoints_6_1 = read_to_waypoints("/home/xht/moveit2_workspace/src/ur3_moveit/config/plan/connect_1/electrical_connect_test_6_1.txt");
	moveit_msgs::msg::RobotTrajectory trajectory_6_1;
	fraction = move_group.computeCartesianPath(waypoints_6_1, eef_step, jump_threshold, trajectory_6_1);
	RCLCPP_INFO(LOGGER, "Visualizing plan Cartesian path (%.2f%% achieved)", fraction * 100.0);
	// 笛卡尔路径的速度规划
	// moveit_msgs::msg::RobotTrajectory trajectory_scaled_1 = scale_trajectory_speed(trajectory_1, 0.01);
	// plan.trajectory_ = trajectory_scaled_1;
	rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory_6_1);
	time_param.computeTimeStamps(rt, 2.5 * 0.001 * 0.8, 0.001); // 0.02对应25mm/s
	rt.getRobotTrajectoryMsg(trajectory_6_1);
	plan.trajectory_ = trajectory_6_1;

	msg_pressure_control.data = 0;  // 示例压力值
	pressure_control_pub->publish(msg_pressure_control);  // 将气压进口打开 

	move_group.execute(plan);

	msg_pressure_control.data = 1;  // 示例压力值
	pressure_control_pub->publish(msg_pressure_control);  // 将气压进口关闭 

	std::vector<geometry_msgs::msg::Pose> waypoints_6_2 = read_to_waypoints("/home/xht/moveit2_workspace/src/ur3_moveit/config/plan/connect_1/electrical_connect_test_6_2.txt");
	moveit_msgs::msg::RobotTrajectory trajectory_6_2;
	fraction = move_group.computeCartesianPath(waypoints_6_2, eef_step, jump_threshold, trajectory_6_2);
	RCLCPP_INFO(LOGGER, "Visualizing plan Cartesian path (%.2f%% achieved)", fraction * 100.0);
	moveit_msgs::msg::RobotTrajectory trajectory_scaled_6_2 = scale_trajectory_speed(trajectory_6_2, kong_scale);
	plan.trajectory_ = trajectory_scaled_6_2;
	move_group.execute(plan);

	// 打印第七段

	std::vector<geometry_msgs::msg::Pose> waypoints_7_1 = read_to_waypoints("/home/xht/moveit2_workspace/src/ur3_moveit/config/plan/connect_1/electrical_connect_test_7_1.txt");
	moveit_msgs::msg::RobotTrajectory trajectory_7_1;
	fraction = move_group.computeCartesianPath(waypoints_7_1, eef_step, jump_threshold, trajectory_7_1);
	RCLCPP_INFO(LOGGER, "Visualizing plan Cartesian path (%.2f%% achieved)", fraction * 100.0);
	// 笛卡尔路径的速度规划
	// moveit_msgs::msg::RobotTrajectory trajectory_scaled_1 = scale_trajectory_speed(trajectory_1, 0.01);
	// plan.trajectory_ = trajectory_scaled_1;
	rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory_7_1);
	time_param.computeTimeStamps(rt, 2.5 * 0.001 * 0.8, 0.001); // 0.02对应25mm/s
	rt.getRobotTrajectoryMsg(trajectory_7_1);
	plan.trajectory_ = trajectory_7_1;

	msg_pressure_control.data = 0;  // 示例压力值
	pressure_control_pub->publish(msg_pressure_control);  // 将气压进口打开 

	move_group.execute(plan);

	msg_pressure_control.data = 1;  // 示例压力值
	pressure_control_pub->publish(msg_pressure_control);  // 将气压进口关闭 

	std::vector<geometry_msgs::msg::Pose> waypoints_7_2 = read_to_waypoints("/home/xht/moveit2_workspace/src/ur3_moveit/config/plan/connect_1/electrical_connect_test_7_2.txt");
	moveit_msgs::msg::RobotTrajectory trajectory_7_2;
	fraction = move_group.computeCartesianPath(waypoints_7_2, eef_step, jump_threshold, trajectory_7_2);
	RCLCPP_INFO(LOGGER, "Visualizing plan Cartesian path (%.2f%% achieved)", fraction * 100.0);
	moveit_msgs::msg::RobotTrajectory trajectory_scaled_7_2 = scale_trajectory_speed(trajectory_7_2, kong_scale);
	plan.trajectory_ = trajectory_scaled_7_2;
	move_group.execute(plan);

	// 打印第八段

	std::vector<geometry_msgs::msg::Pose> waypoints_8_1 = read_to_waypoints("/home/xht/moveit2_workspace/src/ur3_moveit/config/plan/connect_1/electrical_connect_test_8_1.txt");
	moveit_msgs::msg::RobotTrajectory trajectory_8_1;
	fraction = move_group.computeCartesianPath(waypoints_8_1, eef_step, jump_threshold, trajectory_8_1);
	RCLCPP_INFO(LOGGER, "Visualizing plan Cartesian path (%.2f%% achieved)", fraction * 100.0);
	// 笛卡尔路径的速度规划
	// moveit_msgs::msg::RobotTrajectory trajectory_scaled_1 = scale_trajectory_speed(trajectory_1, 0.01);
	// plan.trajectory_ = trajectory_scaled_1;
	rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory_8_1);
	time_param.computeTimeStamps(rt, 2.5 * 0.001 * 0.8, 0.001); // 0.02对应25mm/s
	rt.getRobotTrajectoryMsg(trajectory_8_1);
	plan.trajectory_ = trajectory_8_1;

	msg_pressure_control.data = 0;  // 示例压力值
	pressure_control_pub->publish(msg_pressure_control);  // 将气压进口打开 

	move_group.execute(plan);

	msg_pressure_control.data = 1;  // 示例压力值
	pressure_control_pub->publish(msg_pressure_control);  // 将气压进口关闭 

	std::vector<geometry_msgs::msg::Pose> waypoints_8_2 = read_to_waypoints("/home/xht/moveit2_workspace/src/ur3_moveit/config/plan/connect_1/electrical_connect_test_8_2.txt");
	moveit_msgs::msg::RobotTrajectory trajectory_8_2;
	fraction = move_group.computeCartesianPath(waypoints_8_2, eef_step, jump_threshold, trajectory_8_2);
	RCLCPP_INFO(LOGGER, "Visualizing plan Cartesian path (%.2f%% achieved)", fraction * 100.0);
	moveit_msgs::msg::RobotTrajectory trajectory_scaled_8_2 = scale_trajectory_speed(trajectory_8_2, kong_scale);
	plan.trajectory_ = trajectory_scaled_8_2;
	move_group.execute(plan);

	// 打印第九段

	std::vector<geometry_msgs::msg::Pose> waypoints_9_1 = read_to_waypoints("/home/xht/moveit2_workspace/src/ur3_moveit/config/plan/connect_1/electrical_connect_test_9_1.txt");
	moveit_msgs::msg::RobotTrajectory trajectory_9_1;
	fraction = move_group.computeCartesianPath(waypoints_9_1, eef_step, jump_threshold, trajectory_9_1);
	RCLCPP_INFO(LOGGER, "Visualizing plan Cartesian path (%.2f%% achieved)", fraction * 100.0);
	// 笛卡尔路径的速度规划
	// moveit_msgs::msg::RobotTrajectory trajectory_scaled_1 = scale_trajectory_speed(trajectory_1, 0.01);
	// plan.trajectory_ = trajectory_scaled_1;
	rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory_9_1);
	time_param.computeTimeStamps(rt, 2.5 * 0.001 * 0.8, 0.001); // 0.02对应25mm/s
	rt.getRobotTrajectoryMsg(trajectory_9_1);
	plan.trajectory_ = trajectory_9_1;

	msg_pressure_control.data = 0;  // 示例压力值
	pressure_control_pub->publish(msg_pressure_control);  // 将气压进口打开 

	move_group.execute(plan);

	msg_pressure_control.data = 1;  // 示例压力值
	pressure_control_pub->publish(msg_pressure_control);  // 将气压进口关闭 

	std::vector<geometry_msgs::msg::Pose> waypoints_9_2 = read_to_waypoints("/home/xht/moveit2_workspace/src/ur3_moveit/config/plan/connect_1/electrical_connect_test_9_2.txt");
	moveit_msgs::msg::RobotTrajectory trajectory_9_2;
	fraction = move_group.computeCartesianPath(waypoints_9_2, eef_step, jump_threshold, trajectory_9_2);
	RCLCPP_INFO(LOGGER, "Visualizing plan Cartesian path (%.2f%% achieved)", fraction * 100.0);
	moveit_msgs::msg::RobotTrajectory trajectory_scaled_9_2 = scale_trajectory_speed(trajectory_9_2, kong_scale);
	plan.trajectory_ = trajectory_scaled_9_2;
	move_group.execute(plan);

	// 打印第十段

	std::vector<geometry_msgs::msg::Pose> waypoints_10_1 = read_to_waypoints("/home/xht/moveit2_workspace/src/ur3_moveit/config/plan/connect_1/electrical_connect_test_10_1.txt");
	moveit_msgs::msg::RobotTrajectory trajectory_10_1;
	fraction = move_group.computeCartesianPath(waypoints_10_1, eef_step, jump_threshold, trajectory_10_1);
	RCLCPP_INFO(LOGGER, "Visualizing plan Cartesian path (%.2f%% achieved)", fraction * 100.0);
	// 笛卡尔路径的速度规划
	// moveit_msgs::msg::RobotTrajectory trajectory_scaled_1 = scale_trajectory_speed(trajectory_1, 0.01);
	// plan.trajectory_ = trajectory_scaled_1;
	rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory_10_1);
	time_param.computeTimeStamps(rt, 2.5 * 0.001 * 0.8, 0.001); // 0.02对应25mm/s
	rt.getRobotTrajectoryMsg(trajectory_10_1);
	plan.trajectory_ = trajectory_10_1;

	msg_pressure_control.data = 0;  // 示例压力值
	pressure_control_pub->publish(msg_pressure_control);  // 将气压进口打开 

	move_group.execute(plan);

	msg_pressure_control.data = 1;  // 示例压力值
	pressure_control_pub->publish(msg_pressure_control);  // 将气压进口关闭 

	std::vector<geometry_msgs::msg::Pose> waypoints_10_2 = read_to_waypoints("/home/xht/moveit2_workspace/src/ur3_moveit/config/plan/connect_1/electrical_connect_test_10_2.txt");
	moveit_msgs::msg::RobotTrajectory trajectory_10_2;
	fraction = move_group.computeCartesianPath(waypoints_10_2, eef_step, jump_threshold, trajectory_10_2);
	RCLCPP_INFO(LOGGER, "Visualizing plan Cartesian path (%.2f%% achieved)", fraction * 100.0);
	moveit_msgs::msg::RobotTrajectory trajectory_scaled_10_2 = scale_trajectory_speed(trajectory_10_2, kong_scale);
	plan.trajectory_ = trajectory_scaled_10_2;
	move_group.execute(plan);

	// 打印第十一段

	std::vector<geometry_msgs::msg::Pose> waypoints_11_1 = read_to_waypoints("/home/xht/moveit2_workspace/src/ur3_moveit/config/plan/connect_1/electrical_connect_test_11_1.txt");
	moveit_msgs::msg::RobotTrajectory trajectory_11_1;
	fraction = move_group.computeCartesianPath(waypoints_11_1, eef_step, jump_threshold, trajectory_11_1);
	RCLCPP_INFO(LOGGER, "Visualizing plan Cartesian path (%.2f%% achieved)", fraction * 100.0);
	// 笛卡尔路径的速度规划
	// moveit_msgs::msg::RobotTrajectory trajectory_scaled_1 = scale_trajectory_speed(trajectory_1, 0.01);
	// plan.trajectory_ = trajectory_scaled_1;
	rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory_11_1);
	time_param.computeTimeStamps(rt, 2.5 * 0.001 * 0.8, 0.001); // 0.02对应25mm/s
	rt.getRobotTrajectoryMsg(trajectory_11_1);
	plan.trajectory_ = trajectory_11_1;

	msg_pressure_control.data = 0;  // 示例压力值
	pressure_control_pub->publish(msg_pressure_control);  // 将气压进口打开 

	move_group.execute(plan);

	msg_pressure_control.data = 1;  // 示例压力值
	pressure_control_pub->publish(msg_pressure_control);  // 将气压进口关闭 

	std::vector<geometry_msgs::msg::Pose> waypoints_11_2 = read_to_waypoints("/home/xht/moveit2_workspace/src/ur3_moveit/config/plan/connect_1/electrical_connect_test_11_2.txt");
	moveit_msgs::msg::RobotTrajectory trajectory_11_2;
	fraction = move_group.computeCartesianPath(waypoints_11_2, eef_step, jump_threshold, trajectory_11_2);
	RCLCPP_INFO(LOGGER, "Visualizing plan Cartesian path (%.2f%% achieved)", fraction * 100.0);
	moveit_msgs::msg::RobotTrajectory trajectory_scaled_11_2 = scale_trajectory_speed(trajectory_11_2, kong_scale);
	plan.trajectory_ = trajectory_scaled_11_2;
	move_group.execute(plan);

	// 打印第十二段

	std::vector<geometry_msgs::msg::Pose> waypoints_12_1 = read_to_waypoints("/home/xht/moveit2_workspace/src/ur3_moveit/config/plan/connect_1/electrical_connect_test_12_1.txt");
	moveit_msgs::msg::RobotTrajectory trajectory_12_1;
	fraction = move_group.computeCartesianPath(waypoints_12_1, eef_step, jump_threshold, trajectory_12_1);
	RCLCPP_INFO(LOGGER, "Visualizing plan Cartesian path (%.2f%% achieved)", fraction * 100.0);
	// 笛卡尔路径的速度规划
	// moveit_msgs::msg::RobotTrajectory trajectory_scaled_1 = scale_trajectory_speed(trajectory_1, 0.01);
	// plan.trajectory_ = trajectory_scaled_1;
	rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory_12_1);
	time_param.computeTimeStamps(rt, 2.5 * 0.001 * 0.8, 0.001); // 0.02对应25mm/s
	rt.getRobotTrajectoryMsg(trajectory_12_1);
	plan.trajectory_ = trajectory_12_1;

	msg_pressure_control.data = 0;  // 示例压力值
	pressure_control_pub->publish(msg_pressure_control);  // 将气压进口打开 

	move_group.execute(plan);

	msg_pressure_control.data = 1;  // 示例压力值
	pressure_control_pub->publish(msg_pressure_control);  // 将气压进口关闭 

	std::vector<geometry_msgs::msg::Pose> waypoints_12_2 = read_to_waypoints("/home/xht/moveit2_workspace/src/ur3_moveit/config/plan/connect_1/electrical_connect_test_12_2.txt");
	moveit_msgs::msg::RobotTrajectory trajectory_12_2;
	fraction = move_group.computeCartesianPath(waypoints_12_2, eef_step, jump_threshold, trajectory_12_2);
	RCLCPP_INFO(LOGGER, "Visualizing plan Cartesian path (%.2f%% achieved)", fraction * 100.0);
	moveit_msgs::msg::RobotTrajectory trajectory_scaled_12_2 = scale_trajectory_speed(trajectory_12_2, kong_scale);
	plan.trajectory_ = trajectory_scaled_12_2;
	move_group.execute(plan);

	// 打印第十三段

	std::vector<geometry_msgs::msg::Pose> waypoints_13_1 = read_to_waypoints("/home/xht/moveit2_workspace/src/ur3_moveit/config/plan/connect_1/electrical_connect_test_13_1.txt");
	moveit_msgs::msg::RobotTrajectory trajectory_13_1;
	fraction = move_group.computeCartesianPath(waypoints_13_1, eef_step, jump_threshold, trajectory_13_1);
	RCLCPP_INFO(LOGGER, "Visualizing plan Cartesian path (%.2f%% achieved)", fraction * 100.0);
	// 笛卡尔路径的速度规划
	// moveit_msgs::msg::RobotTrajectory trajectory_scaled_1 = scale_trajectory_speed(trajectory_1, 0.01);
	// plan.trajectory_ = trajectory_scaled_1;
	rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory_13_1);
	time_param.computeTimeStamps(rt, 2.5 * 0.001 * 0.8, 0.001); // 0.02对应25mm/s
	rt.getRobotTrajectoryMsg(trajectory_13_1);
	plan.trajectory_ = trajectory_13_1;

	msg_pressure_control.data = 0;  // 示例压力值
	pressure_control_pub->publish(msg_pressure_control);  // 将气压进口打开 

	move_group.execute(plan);

	msg_pressure_control.data = 1;  // 示例压力值
	pressure_control_pub->publish(msg_pressure_control);  // 将气压进口关闭 

	std::vector<geometry_msgs::msg::Pose> waypoints_13_2 = read_to_waypoints("/home/xht/moveit2_workspace/src/ur3_moveit/config/plan/connect_1/electrical_connect_test_13_2.txt");
	moveit_msgs::msg::RobotTrajectory trajectory_13_2;
	fraction = move_group.computeCartesianPath(waypoints_13_2, eef_step, jump_threshold, trajectory_13_2);
	RCLCPP_INFO(LOGGER, "Visualizing plan Cartesian path (%.2f%% achieved)", fraction * 100.0);
	moveit_msgs::msg::RobotTrajectory trajectory_scaled_13_2 = scale_trajectory_speed(trajectory_13_2, kong_scale);
	plan.trajectory_ = trajectory_scaled_13_2;
	move_group.execute(plan);

	// 打印第十四段

	std::vector<geometry_msgs::msg::Pose> waypoints_14_1 = read_to_waypoints("/home/xht/moveit2_workspace/src/ur3_moveit/config/plan/connect_1/electrical_connect_test_14_1.txt");
	moveit_msgs::msg::RobotTrajectory trajectory_14_1;
	fraction = move_group.computeCartesianPath(waypoints_14_1, eef_step, jump_threshold, trajectory_14_1);
	RCLCPP_INFO(LOGGER, "Visualizing plan Cartesian path (%.2f%% achieved)", fraction * 100.0);
	// 笛卡尔路径的速度规划
	// moveit_msgs::msg::RobotTrajectory trajectory_scaled_1 = scale_trajectory_speed(trajectory_1, 0.01);
	// plan.trajectory_ = trajectory_scaled_1;
	rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory_14_1);
	time_param.computeTimeStamps(rt, 2.5 * 0.001 * 0.8, 0.001); // 0.02对应25mm/s
	rt.getRobotTrajectoryMsg(trajectory_14_1);
	plan.trajectory_ = trajectory_14_1;

	msg_pressure_control.data = 0;  // 示例压力值
	pressure_control_pub->publish(msg_pressure_control);  // 将气压进口打开 

	move_group.execute(plan);

	msg_pressure_control.data = 1;  // 示例压力值
	pressure_control_pub->publish(msg_pressure_control);  // 将气压进口关闭 

	std::vector<geometry_msgs::msg::Pose> waypoints_14_2 = read_to_waypoints("/home/xht/moveit2_workspace/src/ur3_moveit/config/plan/connect_1/electrical_connect_test_14_2.txt");
	moveit_msgs::msg::RobotTrajectory trajectory_14_2;
	fraction = move_group.computeCartesianPath(waypoints_14_2, eef_step, jump_threshold, trajectory_14_2);
	RCLCPP_INFO(LOGGER, "Visualizing plan Cartesian path (%.2f%% achieved)", fraction * 100.0);
	moveit_msgs::msg::RobotTrajectory trajectory_scaled_14_2 = scale_trajectory_speed(trajectory_14_2, kong_scale);
	plan.trajectory_ = trajectory_scaled_14_2;
	move_group.execute(plan);

	// 打印第十五段

	std::vector<geometry_msgs::msg::Pose> waypoints_15_1 = read_to_waypoints("/home/xht/moveit2_workspace/src/ur3_moveit/config/plan/connect_1/electrical_connect_test_15_1.txt");
	moveit_msgs::msg::RobotTrajectory trajectory_15_1;
	fraction = move_group.computeCartesianPath(waypoints_15_1, eef_step, jump_threshold, trajectory_15_1);
	RCLCPP_INFO(LOGGER, "Visualizing plan Cartesian path (%.2f%% achieved)", fraction * 100.0);
	// 笛卡尔路径的速度规划
	// moveit_msgs::msg::RobotTrajectory trajectory_scaled_1 = scale_trajectory_speed(trajectory_1, 0.01);
	// plan.trajectory_ = trajectory_scaled_1;
	rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory_15_1);
	time_param.computeTimeStamps(rt, 2.5 * 0.001 * 0.8, 0.001); // 0.02对应25mm/s
	rt.getRobotTrajectoryMsg(trajectory_15_1);
	plan.trajectory_ = trajectory_15_1;

	msg_pressure_control.data = 0;  // 示例压力值
	pressure_control_pub->publish(msg_pressure_control);  // 将气压进口打开 

	move_group.execute(plan);

	msg_pressure_control.data = 1;  // 示例压力值
	pressure_control_pub->publish(msg_pressure_control);  // 将气压进口关闭 

	std::vector<geometry_msgs::msg::Pose> waypoints_15_2 = read_to_waypoints("/home/xht/moveit2_workspace/src/ur3_moveit/config/plan/connect_1/electrical_connect_test_15_2.txt");
	moveit_msgs::msg::RobotTrajectory trajectory_15_2;
	fraction = move_group.computeCartesianPath(waypoints_15_2, eef_step, jump_threshold, trajectory_15_2);
	RCLCPP_INFO(LOGGER, "Visualizing plan Cartesian path (%.2f%% achieved)", fraction * 100.0);
	moveit_msgs::msg::RobotTrajectory trajectory_scaled_15_2 = scale_trajectory_speed(trajectory_15_2, kong_scale);
	plan.trajectory_ = trajectory_scaled_15_2;
	move_group.execute(plan);

	// 打印第十六段

	std::vector<geometry_msgs::msg::Pose> waypoints_16_1 = read_to_waypoints("/home/xht/moveit2_workspace/src/ur3_moveit/config/plan/connect_1/electrical_connect_test_16_1.txt");
	moveit_msgs::msg::RobotTrajectory trajectory_16_1;
	fraction = move_group.computeCartesianPath(waypoints_16_1, eef_step, jump_threshold, trajectory_16_1);
	RCLCPP_INFO(LOGGER, "Visualizing plan Cartesian path (%.2f%% achieved)", fraction * 100.0);
	// 笛卡尔路径的速度规划
	// moveit_msgs::msg::RobotTrajectory trajectory_scaled_1 = scale_trajectory_speed(trajectory_1, 0.01);
	// plan.trajectory_ = trajectory_scaled_1;
	rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory_16_1);
	time_param.computeTimeStamps(rt, 2.5 * 0.001 * 0.8, 0.001); // 0.02对应25mm/s
	rt.getRobotTrajectoryMsg(trajectory_16_1);
	plan.trajectory_ = trajectory_16_1;

	msg_pressure_control.data = 0;  // 示例压力值
	pressure_control_pub->publish(msg_pressure_control);  // 将气压进口打开 

	move_group.execute(plan);

	msg_pressure_control.data = 1;  // 示例压力值
	pressure_control_pub->publish(msg_pressure_control);  // 将气压进口关闭 

	std::vector<geometry_msgs::msg::Pose> waypoints_16_2 = read_to_waypoints("/home/xht/moveit2_workspace/src/ur3_moveit/config/plan/connect_1/electrical_connect_test_16_2.txt");
	moveit_msgs::msg::RobotTrajectory trajectory_16_2;
	fraction = move_group.computeCartesianPath(waypoints_16_2, eef_step, jump_threshold, trajectory_16_2);
	RCLCPP_INFO(LOGGER, "Visualizing plan Cartesian path (%.2f%% achieved)", fraction * 100.0);
	moveit_msgs::msg::RobotTrajectory trajectory_scaled_16_2 = scale_trajectory_speed(trajectory_16_2, kong_scale);
	plan.trajectory_ = trajectory_scaled_16_2;
	move_group.execute(plan);

	// 打印第十七段

	std::vector<geometry_msgs::msg::Pose> waypoints_17_1 = read_to_waypoints("/home/xht/moveit2_workspace/src/ur3_moveit/config/plan/connect_1/electrical_connect_test_17_1.txt");
	moveit_msgs::msg::RobotTrajectory trajectory_17_1;
	fraction = move_group.computeCartesianPath(waypoints_17_1, eef_step, jump_threshold, trajectory_17_1);
	RCLCPP_INFO(LOGGER, "Visualizing plan Cartesian path (%.2f%% achieved)", fraction * 100.0);
	// 笛卡尔路径的速度规划
	// moveit_msgs::msg::RobotTrajectory trajectory_scaled_1 = scale_trajectory_speed(trajectory_1, 0.01);
	// plan.trajectory_ = trajectory_scaled_1;
	rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory_17_1);
	time_param.computeTimeStamps(rt, 2.5 * 0.001 * 0.8, 0.001); // 0.02对应25mm/s
	rt.getRobotTrajectoryMsg(trajectory_17_1);
	plan.trajectory_ = trajectory_17_1;

	msg_pressure_control.data = 0;  // 示例压力值
	pressure_control_pub->publish(msg_pressure_control);  // 将气压进口打开 

	move_group.execute(plan);

	msg_pressure_control.data = 1;  // 示例压力值
	pressure_control_pub->publish(msg_pressure_control);  // 将气压进口关闭 

	std::vector<geometry_msgs::msg::Pose> waypoints_17_2 = read_to_waypoints("/home/xht/moveit2_workspace/src/ur3_moveit/config/plan/connect_1/electrical_connect_test_17_2.txt");
	moveit_msgs::msg::RobotTrajectory trajectory_17_2;
	fraction = move_group.computeCartesianPath(waypoints_17_2, eef_step, jump_threshold, trajectory_17_2);
	RCLCPP_INFO(LOGGER, "Visualizing plan Cartesian path (%.2f%% achieved)", fraction * 100.0);
	moveit_msgs::msg::RobotTrajectory trajectory_scaled_17_2 = scale_trajectory_speed(trajectory_17_2, kong_scale);
	plan.trajectory_ = trajectory_scaled_17_2;
	move_group.execute(plan);

	// 打印第十八段

	std::vector<geometry_msgs::msg::Pose> waypoints_18_1 = read_to_waypoints("/home/xht/moveit2_workspace/src/ur3_moveit/config/plan/connect_1/electrical_connect_test_18_1.txt");
	moveit_msgs::msg::RobotTrajectory trajectory_18_1;
	fraction = move_group.computeCartesianPath(waypoints_18_1, eef_step, jump_threshold, trajectory_18_1);
	RCLCPP_INFO(LOGGER, "Visualizing plan Cartesian path (%.2f%% achieved)", fraction * 100.0);
	// 笛卡尔路径的速度规划
	// moveit_msgs::msg::RobotTrajectory trajectory_scaled_1 = scale_trajectory_speed(trajectory_1, 0.01);
	// plan.trajectory_ = trajectory_scaled_1;
	rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory_18_1);
	time_param.computeTimeStamps(rt, 2.5 * 0.001 * 0.8, 0.001); // 0.02对应25mm/s
	rt.getRobotTrajectoryMsg(trajectory_18_1);
	plan.trajectory_ = trajectory_18_1;

	msg_pressure_control.data = 0;  // 示例压力值
	pressure_control_pub->publish(msg_pressure_control);  // 将气压进口打开 

	move_group.execute(plan);

	msg_pressure_control.data = 1;  // 示例压力值
	pressure_control_pub->publish(msg_pressure_control);  // 将气压进口关闭 

	std::vector<geometry_msgs::msg::Pose> waypoints_18_2 = read_to_waypoints("/home/xht/moveit2_workspace/src/ur3_moveit/config/plan/connect_1/electrical_connect_test_18_2.txt");
	moveit_msgs::msg::RobotTrajectory trajectory_18_2;
	fraction = move_group.computeCartesianPath(waypoints_18_2, eef_step, jump_threshold, trajectory_18_2);
	RCLCPP_INFO(LOGGER, "Visualizing plan Cartesian path (%.2f%% achieved)", fraction * 100.0);
	moveit_msgs::msg::RobotTrajectory trajectory_scaled_18_2 = scale_trajectory_speed(trajectory_18_2, kong_scale);
	plan.trajectory_ = trajectory_scaled_18_2;
	move_group.execute(plan); 

	// 打印第十九段

	std::vector<geometry_msgs::msg::Pose> waypoints_19_1 = read_to_waypoints("/home/xht/moveit2_workspace/src/ur3_moveit/config/plan/connect_1/electrical_connect_test_19_1.txt");
	moveit_msgs::msg::RobotTrajectory trajectory_19_1;
	fraction = move_group.computeCartesianPath(waypoints_19_1, eef_step, jump_threshold, trajectory_19_1);
	RCLCPP_INFO(LOGGER, "Visualizing plan Cartesian path (%.2f%% achieved)", fraction * 100.0);
	// 笛卡尔路径的速度规划
	// moveit_msgs::msg::RobotTrajectory trajectory_scaled_1 = scale_trajectory_speed(trajectory_1, 0.01);
	// plan.trajectory_ = trajectory_scaled_1;
	rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory_19_1);
	time_param.computeTimeStamps(rt, 2.5 * 0.001 * 0.8, 0.001); // 0.02对应25mm/s
	rt.getRobotTrajectoryMsg(trajectory_19_1);
	plan.trajectory_ = trajectory_19_1;

	msg_pressure_control.data = 0;  // 示例压力值
	pressure_control_pub->publish(msg_pressure_control);  // 将气压进口打开 

	move_group.execute(plan);

	msg_pressure_control.data = 1;  // 示例压力值
	pressure_control_pub->publish(msg_pressure_control);  // 将气压进口关闭

	std::vector<geometry_msgs::msg::Pose> waypoints_19_2 = read_to_waypoints("/home/xht/moveit2_workspace/src/ur3_moveit/config/plan/connect_1/electrical_connect_test_19_2.txt");
	moveit_msgs::msg::RobotTrajectory trajectory_19_2;
	fraction = move_group.computeCartesianPath(waypoints_19_2, eef_step, jump_threshold, trajectory_19_2);
	RCLCPP_INFO(LOGGER, "Visualizing plan Cartesian path (%.2f%% achieved)", fraction * 100.0);
	moveit_msgs::msg::RobotTrajectory trajectory_scaled_19_2 = scale_trajectory_speed(trajectory_19_2, kong_scale);
	plan.trajectory_ = trajectory_scaled_19_2;
	move_group.execute(plan);

	// 打印第二十段

	std::vector<geometry_msgs::msg::Pose> waypoints_20_1 = read_to_waypoints("/home/xht/moveit2_workspace/src/ur3_moveit/config/plan/connect_1/electrical_connect_test_20_1.txt");
	moveit_msgs::msg::RobotTrajectory trajectory_20_1;
	fraction = move_group.computeCartesianPath(waypoints_20_1, eef_step, jump_threshold, trajectory_20_1);
	RCLCPP_INFO(LOGGER, "Visualizing plan Cartesian path (%.2f%% achieved)", fraction * 100.0);
	// 笛卡尔路径的速度规划
	// moveit_msgs::msg::RobotTrajectory trajectory_scaled_1 = scale_trajectory_speed(trajectory_1, 0.01);
	// plan.trajectory_ = trajectory_scaled_1;
	rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory_20_1);
	time_param.computeTimeStamps(rt, 2.5 * 0.001 * 0.8, 0.001); // 0.02对应25mm/s
	rt.getRobotTrajectoryMsg(trajectory_20_1);
	plan.trajectory_ = trajectory_20_1;

	msg_pressure_control.data = 0;  // 示例压力值
	pressure_control_pub->publish(msg_pressure_control);  // 将气压进口打开 

	move_group.execute(plan);

	msg_pressure_control.data = 1;  // 示例压力值
	pressure_control_pub->publish(msg_pressure_control);  // 将气压进口关闭

	std::vector<geometry_msgs::msg::Pose> waypoints_20_2 = read_to_waypoints("/home/xht/moveit2_workspace/src/ur3_moveit/config/plan/connect_1/electrical_connect_test_20_2.txt");
	moveit_msgs::msg::RobotTrajectory trajectory_20_2;
	fraction = move_group.computeCartesianPath(waypoints_20_2, eef_step, jump_threshold, trajectory_20_2);
	RCLCPP_INFO(LOGGER, "Visualizing plan Cartesian path (%.2f%% achieved)", fraction * 100.0);
	moveit_msgs::msg::RobotTrajectory trajectory_scaled_20_2 = scale_trajectory_speed(trajectory_20_2, kong_scale);
	plan.trajectory_ = trajectory_scaled_20_2;
	move_group.execute(plan);

	// 打印第二十一段

	std::vector<geometry_msgs::msg::Pose> waypoints_21_1 = read_to_waypoints("/home/xht/moveit2_workspace/src/ur3_moveit/config/plan/connect_1/electrical_connect_test_21_1.txt");
	moveit_msgs::msg::RobotTrajectory trajectory_21_1;
	fraction = move_group.computeCartesianPath(waypoints_21_1, eef_step, jump_threshold, trajectory_21_1);
	RCLCPP_INFO(LOGGER, "Visualizing plan Cartesian path (%.2f%% achieved)", fraction * 100.0);
	// 笛卡尔路径的速度规划
	// moveit_msgs::msg::RobotTrajectory trajectory_scaled_1 = scale_trajectory_speed(trajectory_1, 0.01);
	// plan.trajectory_ = trajectory_scaled_1;
	rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory_21_1);
	time_param.computeTimeStamps(rt, 2.5 * 0.001 * 0.8, 0.001); // 0.02对应25mm/s
	rt.getRobotTrajectoryMsg(trajectory_21_1);
	plan.trajectory_ = trajectory_21_1;

	msg_pressure_control.data = 0;  // 示例压力值
	pressure_control_pub->publish(msg_pressure_control);  // 将气压进口打开 

	move_group.execute(plan);

	msg_pressure_control.data = 1;  // 示例压力值
	msg_pressure_adjust.data = 0;  // 示例压力值
	pressure_control_pub->publish(msg_pressure_control);  // 将气压进口关闭
	pressure_adjust_pub->publish(msg_pressure_adjust);

	std::vector<geometry_msgs::msg::Pose> waypoints_21_2 = read_to_waypoints("/home/xht/moveit2_workspace/src/ur3_moveit/config/plan/connect_1/electrical_connect_test_21_2.txt");
	moveit_msgs::msg::RobotTrajectory trajectory_21_2;
	fraction = move_group.computeCartesianPath(waypoints_21_2, eef_step, jump_threshold, trajectory_21_2);
	RCLCPP_INFO(LOGGER, "Visualizing plan Cartesian path (%.2f%% achieved)", fraction * 100.0);
	moveit_msgs::msg::RobotTrajectory trajectory_scaled_21_2 = scale_trajectory_speed(trajectory_21_2, kong_scale);
	plan.trajectory_ = trajectory_scaled_21_2;
	move_group.execute(plan);


	// API_set_laser_on()
	visual_tools.deleteAllMarkers();
	visual_tools.trigger();
	rclcpp::shutdown();
	return 0;
}


