/**
 * @file pilz_ur3_moveit.cpp
 * @brief 基于MoveIt的UR3机械臂控制程序，实现笛卡尔路径规划和3D打印控制
 * 
 * 本程序主要功能：
 * - 初始化MoveIt控制接口
 * - 读取配置文件中的TCP工具参数
 * - 从文件加载打印路径规划
 * - 执行速度调整后的轨迹规划
 * - 集成气压控制系统接口
 */

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

#include <moveit_msgs/msg/motion_sequence_item.hpp>
#include <moveit_msgs/msg/motion_sequence_request.hpp>
#include <moveit_msgs/msg/planning_options.hpp>
#include <moveit_msgs/srv/get_motion_sequence.hpp>
#include <moveit_msgs/action/move_group_sequence.hpp>

#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit_msgs/msg/motion_plan_response.hpp>
#include <moveit/kinematic_constraints/utils.h>

#include <moveit_msgs/srv/get_motion_sequence.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

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

static const rclcpp::Logger LOGGER = rclcpp::get_logger("node name: pilz_ur_moveit");
using moveit_msgs::action::MoveGroupSequence;
using GoalHandleMoveGroupSequence = rclcpp_action::ClientGoalHandle<MoveGroupSequence>;

/**
 * @brief 调整轨迹速度
 * @param trajectory 原始轨迹数据
 * @param scale 速度缩放因子 (0.0-1.0)
 * @return 调整后的轨迹数据
 * 
 * 该函数通过以下方式调整轨迹：
 * 1. 调整时间间隔
 * 2. 缩放关节速度
 * 3. 调整加速度
 */

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
	// 初始化ROS2节点和MoveIt接口
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions node_options;
	node_options.automatically_declare_parameters_from_overrides(true);
	auto move_group_node = rclcpp::Node::make_shared("pilz_ur_moveit", node_options);

	rclcpp::executors::SingleThreadedExecutor executor;
	executor.add_node(move_group_node);
	std::thread([&executor]() { executor.spin(); }).detach();

	// 设置机械臂规划组和可视化工具
	static const std::string PLANNING_GROUP = "ur_manipulator";  // 对应MoveIt配置中的规划组名称

	// 创建气压控制相关的发布器
	auto pressure_control_pub = move_group_node->create_publisher<std_msgs::msg::Int32>("/pressure_control", 10);  // 气压开关控制
	auto pressure_adjust_pub = move_group_node->create_publisher<std_msgs::msg::Int32>("/pressure_adjust", 10);    // 气压值调整
	
	// 创建MoveIt MoveGroup Interface
	moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;



	// 初始化可视化工具
	namespace rvt = rviz_visual_tools;
	moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "world", "move_group_tutorial",
														move_group.getRobotModel());

	visual_tools.deleteAllMarkers();
	visual_tools.loadRemoteControl();


	Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
	text_pose.translation().z() = 1.0;
	visual_tools.publishText(text_pose, "MoveGroupInterface_Demo", rvt::WHITE, rvt::XLARGE);
	visual_tools.trigger();

	// 打印关键配置信息
	RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());  // 输出规划参考坐标系
	RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());
	RCLCPP_INFO(LOGGER, "Available Planning Groups:");
	std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
				std::ostream_iterator<std::string>(std::cout, ", "));

	// Start the demo
	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to initialize the workspace");

	// Create closures for visualization
	auto const draw_title = [&visual_tools](const auto& text) {
		auto const text_pose = [] {
		auto msg = Eigen::Isometry3d::Identity();
		msg.translation().z() = 1.0;
		return msg;
		}();
		visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
	};
	auto const prompt = [&visual_tools](const auto& text) { visual_tools.prompt(text); };
	auto const draw_trajectory_tool_path = [&](const std::vector<moveit_msgs::msg::RobotTrajectory>& trajectories) {
		for (const auto& trajectory : trajectories) {
			visual_tools.publishTrajectoryLine(trajectory, move_group.getRobotModel()->getJointModelGroup(PLANNING_GROUP));  // 正确：逐个传递单个轨迹
		}
	};

	// auto const draw_trajectory_tool_path =
	// 	[&visual_tools,
	// 	jmg = move_group.getRobotModel()->getJointModelGroup(PLANNING_GROUP)](auto const& trajectories) {
	// 	for (const auto& trajectory : trajectories)
	// 	{
	// 		visual_tools.publishTrajectoryLine(trajectory, jmg);
	// 	}
	// };


	move_group.setPlanningPipelineId("pilz_industrial_motion_planner");
	const moveit::core::JointModelGroup* joint_model_group =
		move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
	std::string end_effector_name = joint_model_group->getLinkModelNames().back();



	moveit_msgs::msg::MotionSequenceRequest sequence_request;

	// 第一个目标点
	moveit_msgs::msg::MotionSequenceItem item1;
	item1.blend_radius = 0.01; // 设置混合半径
	item1.req.group_name = PLANNING_GROUP; // 替换为你的规划组
	item1.req.planner_id = "PTP"; // 使用线性规划
	item1.req.allowed_planning_time = 10.0;
	item1.req.max_velocity_scaling_factor = 0.1;
	item1.req.max_acceleration_scaling_factor = 0.1;

	// 设置目标姿态
	geometry_msgs::msg::PoseStamped pose1;
	pose1.header.frame_id = "world";
	pose1.pose.position.x = -0.375;
	pose1.pose.position.y = 0.0565;
	pose1.pose.position.z = 0.2;
	pose1.pose.orientation.x = 1.0;
	pose1.pose.orientation.y = 0.0;
	pose1.pose.orientation.z = 0.0;
	pose1.pose.orientation.w = 0.0;
	item1.req.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(end_effector_name, pose1));

	sequence_request.items.push_back(item1);

	// 第二个目标点
	moveit_msgs::msg::MotionSequenceItem item2;
	item2.blend_radius = 0.01;
	item2.req.group_name = PLANNING_GROUP;
	item2.req.planner_id = "LIN";
	item2.req.allowed_planning_time = 10.0;
	item2.req.max_velocity_scaling_factor = 0.1;
	item2.req.max_acceleration_scaling_factor = 0.1;

	geometry_msgs::msg::PoseStamped pose2;
	pose2.header.frame_id = "world";
	pose2.pose.position.x = -0.375;
	pose2.pose.position.y = 0.211;
	pose2.pose.position.z = 0.2;
	pose2.pose.orientation.x = 1.0;
	pose2.pose.orientation.y = 0.0;
	pose2.pose.orientation.z = 0.0;
	pose2.pose.orientation.w = 0.0;
	item2.req.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(end_effector_name, pose2));

	sequence_request.items.push_back(item2);

	// 发送请求
	using GetMotionSequence = moveit_msgs::srv::GetMotionSequence;
	auto client = move_group_node->create_client<moveit_msgs::srv::GetMotionSequence>("/plan_sequence_path");
	// while (!client->wait_for_service(std::chrono::seconds(10)))
	// {
	// 	RCLCPP_WARN(move_group_node->get_logger(), "Waiting for service /plan_sequence_path to be available...");
	// }

	auto request = std::make_shared<moveit_msgs::srv::GetMotionSequence::Request>();
	request->request = sequence_request;

	auto service_future = client->async_send_request(request);
  

	// Wait for the result
	std::future_status service_status;
	do
	{
	  switch (service_status = service_future.wait_for(std::chrono::seconds(1)); service_status)
	  {
		case std::future_status::deferred:
		  RCLCPP_ERROR(LOGGER, "Deferred");
		  break;
		case std::future_status::timeout:
		  RCLCPP_INFO(LOGGER, "Waiting for trajectory plan...");
		  break;
		case std::future_status::ready:
		  RCLCPP_INFO(LOGGER, "Service ready!");
		  break;
	  }
	} while (service_status != std::future_status::ready);

	auto service_response = service_future.get();
	if (service_response->response.error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
	{
		RCLCPP_INFO(LOGGER, "Planning successful");

		// Access the planned trajectory
		auto trajectory = service_response->response.planned_trajectories;
		draw_trajectory_tool_path(trajectory);
		visual_tools.trigger();
	}
	else
	{
		RCLCPP_ERROR(LOGGER, "Planning failed with error code: %d", service_response->response.error_code.val);

		rclcpp::shutdown();
		return 0;
	}

	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // [ --------------------------------------------------------------- ]
  // [ ------------------ MoveGroupSequence Action ------------------- ]
  // [ --------------------------------------------------------------- ]
  // Plans and executes the trajectory

  // MoveGroupSequence action client
  using MoveGroupSequence = moveit_msgs::action::MoveGroupSequence;
  auto action_client = rclcpp_action::create_client<MoveGroupSequence>(move_group_node, "/sequence_move_group");

  // Verify that the action server is up and running
  if (!action_client->wait_for_action_server(std::chrono::seconds(10)))
  {
    RCLCPP_ERROR(LOGGER, "Error waiting for action server /sequence_move_group");
    return -1;
  }

  // Create action goal
  auto goal_msg = MoveGroupSequence::Goal();
  goal_msg.request = sequence_request;

  // Planning options
  goal_msg.planning_options.planning_scene_diff.is_diff = true;
  goal_msg.planning_options.planning_scene_diff.robot_state.is_diff = true;
  // goal_msg.planning_options.plan_only = true; // Uncomment to only plan the trajectory

  // Goal response callback
  auto send_goal_options = rclcpp_action::Client<MoveGroupSequence>::SendGoalOptions();
  send_goal_options.goal_response_callback = [](std::shared_ptr<GoalHandleMoveGroupSequence> goal_handle) {
    try
    {
      if (!goal_handle)
      {
        RCLCPP_ERROR(LOGGER, "Goal was rejected by server");
      }
      else
      {
        RCLCPP_INFO(LOGGER, "Goal accepted by server, waiting for result");
      }
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(LOGGER, "Exception while waiting for goal response: %s", e.what());
    }
  };

  // Result callback
  send_goal_options.result_callback = [](const GoalHandleMoveGroupSequence::WrappedResult& result) {
    switch (result.code)
    {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(LOGGER, "Goal succeeded");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(LOGGER, "Goal was aborted. Status: %d", result.result->response.error_code.val);
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(LOGGER, "Goal was canceled");
        break;
      default:
        RCLCPP_ERROR(LOGGER, "Unknown result code");
        break;
    }
    RCLCPP_INFO(LOGGER, "Result received");
  };

  // Send the action goal
  auto goal_handle_future = action_client->async_send_goal(goal_msg, send_goal_options);

  // Get result
  auto action_result_future = action_client->async_get_result(goal_handle_future.get());

  // Wait for the result
	std::future_status action_status;
	do
	{
		switch (action_status = action_result_future.wait_for(std::chrono::seconds(1)); action_status)
		{
		case std::future_status::deferred:
			RCLCPP_ERROR(LOGGER, "Deferred");
			break;
		case std::future_status::timeout:
			RCLCPP_INFO(LOGGER, "Executing trajectory...");
			break;
		case std::future_status::ready:
			RCLCPP_INFO(LOGGER, "Action ready!");
			break;
		}
	} while (action_status != std::future_status::ready);

	if (action_result_future.valid())
	{
		auto result = action_result_future.get();
		RCLCPP_INFO(LOGGER, "Action completed. Result: %d", static_cast<int>(result.code));
	}
	else
	{
		RCLCPP_ERROR(LOGGER, "Action couldn't be completed.");
	}


	  


	// if (future.wait_for(std::chrono::seconds(10)) == std::future_status::ready)
	// {
	// 	auto response = future.get();
	// 	if (response->response.error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
	// 	{
	// 		RCLCPP_INFO(move_group_node->get_logger(), "Planning successful!");
	// 	}
	// 	else
	// 	{
	// 		RCLCPP_ERROR(move_group_node->get_logger(), "Planning failed with error code: %d", response->response.error_code.val);
	// 	}
	// }
	// else
	// {
	// 	RCLCPP_ERROR(move_group_node->get_logger(), "Failed to get response from service.");
	// }




	//  std::vector<geometry_msgs::msg::PoseStamped> ptp_targets = {
	// 	 [] { // Target 1
	// 	   geometry_msgs::msg::PoseStamped msg;
	// 	   msg.header.frame_id = "world";
	// 	   msg.pose.orientation.x = 1.0;
	// 	   msg.pose.orientation.y = 0.0;
	// 	   msg.pose.orientation.z = 0.0;
	// 	   msg.pose.orientation.w = 0.0;
	// 	   msg.pose.position.x = -0.375;
	// 	   msg.pose.position.y = 0.0565;
	// 	   msg.pose.position.z = 0.2;
	// 	   return msg;
	// 	 }(),
	// 	 [] { // Target 2
	// 	   geometry_msgs::msg::PoseStamped msg;
	// 	   msg.header.frame_id = "world";
	// 	   msg.pose.orientation.x = 1.0;
	// 	   msg.pose.orientation.y = 0.0;
	// 	   msg.pose.orientation.z = 0.0;
	// 	   msg.pose.orientation.w = 0.0;
	// 	   msg.pose.position.x = -0.375;
	// 	   msg.pose.position.y = 0.211;
	// 	   msg.pose.position.z = 0.2;
	// 	   return msg;
	// 	 }(),
	// 	 [] { // Target 3
	// 	   geometry_msgs::msg::PoseStamped msg;
	// 	   msg.header.frame_id = "world";
	// 	   msg.pose.orientation.x = 1.0;
	// 	   msg.pose.orientation.y = 0.0;
	// 	   msg.pose.orientation.z = 0.0;
	// 	   msg.pose.orientation.w = 0.0;
	// 	   msg.pose.position.x = -0.375;
	// 	   msg.pose.position.y = -0.089;
	// 	   msg.pose.position.z = 0.2;
	// 	   return msg;
	// 	 }()
	//    };
	
	//    move_group.setPlannerId("PTP");
	//    move_group.setMaxVelocityScalingFactor(0.005);
	//    move_group.setMaxAccelerationScalingFactor(0.005);
	//    move_group.clearPathConstraints();

	//    geometry_msgs::msg::Pose target_pose, visual_pose;
	// std::vector<geometry_msgs::msg::Pose> waypoints_first;

	// target_pose.position.x = -0.375;
	// target_pose.position.y = 0.211;
	// target_pose.position.z = 0.20;
	// target_pose.orientation.x = 1.0;
	// target_pose.orientation.y = 0.0;
	// target_pose.orientation.z = 0.0;
	// target_pose.orientation.w = 0.0;

	// waypoints_first.push_back(target_pose);

	// target_pose.position.x = -0.375;
	// target_pose.position.y = 0.0565;
	// target_pose.position.z = 0.20;
	// target_pose.orientation.x = 1.0;
	// target_pose.orientation.y = 0.0;
	// target_pose.orientation.z = 0.0;
	// target_pose.orientation.w = 0.0;

	// waypoints_first.push_back(target_pose);
	
	// moveit_msgs::msg::RobotTrajectory trajectory_first;
	// const double jump_threshold = 0.0;
	// const double eef_step = 0.01;
	// double fraction_first = move_group.computeCartesianPath(waypoints_first, eef_step, jump_threshold, trajectory_first);

	// moveit_msgs::msg::RobotTrajectory new_trajectory_first;
	// new_trajectory_first = scale_trajectory_speed(trajectory_first, 1.0);
	// move_group.execute(new_trajectory_first);

	
	//    // Execute PTP motions sequentially
	//    for (size_t i = 0; i < ptp_targets.size(); ++i) {
	// 		move_group.setPoseTarget(ptp_targets[i], end_effector_name);
	// 	 	plan_and_execute("[PTP] Point " + std::to_string(i+1));
	//    }



	// API_set_laser_on()
	visual_tools.deleteAllMarkers();
	visual_tools.trigger();
	rclcpp::shutdown();
	return 0;
}
