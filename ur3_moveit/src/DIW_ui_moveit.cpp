#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>
#include <thread>
#include <iomanip>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "my_custom_msgs/msg/diw_ui_print.hpp"
#include "my_custom_msgs/msg/trajectory_output.hpp"
#include "my_custom_msgs/msg/test_forward_position_controller.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("DIW_ui_moveit");
double offset_x, offset_y, offset_z;
double kong_scale = 0.06, go_scale = 0.001;
nav_msgs::msg::Path path;

std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pressure_control_pub;
rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pressure_adjust_pub;
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_correct_distance_pub;
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr whether_distance_collect_pub;
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_expend_kalman_filter_pub;
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_torque_correction_pub;
rclcpp::Node::SharedPtr move_group_node;
static const std::string PLANNING_GROUP = "ur_manipulator";

// test_forward_position_controller参数
size_t current_point_index_ = 0;
rclcpp::TimerBase::SharedPtr timer_;
rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_publisher_;
moveit_msgs::msg::RobotTrajectory trajectory_test_forward_position_controller;
std::shared_ptr<rclcpp::Client<controller_manager_msgs::srv::SwitchController>> switch_controller_client;

// 定义一个函数来计算雅可比矩阵
Eigen::MatrixXd fastJacob(Eigen::VectorXd q) {
    // UR3 机械臂参数
    const double a2 = -243.65;
    const double a3 = -213.0;
    const double d1 = 151.9;
    const double d2 = 119.85;
    const double d4 = -9.45;
    const double d5 = 83.4;
    const double d6 = 82.4;

    // 初始化雅可比矩阵
    Eigen::MatrixXd J(6, 6);
    J.setZero();

    // 计算雅可比矩阵的每个元素
    J(0,0) = d6 * (cos(q(0)) * cos(q(4)) + cos(q(1) + q(2) + q(3)) * sin(q(0)) * sin(q(4))) + d2 * cos(q(0)) + d4 * cos(q(0)) - a3 * cos(q(1) + q(2)) * sin(q(0)) - a2 * cos(q(1)) * sin(q(0)) - d5 * sin(q(1) + q(2) + q(3)) * sin(q(0));
    J(1,0) = d6 * (cos(q(4)) * sin(q(0)) - cos(q(1) + q(2) + q(3)) * cos(q(0)) * sin(q(4))) + d2 * sin(q(0)) + d4 * sin(q(0)) + a3 * cos(q(1) + q(2)) * cos(q(0)) + a2 * cos(q(0)) * cos(q(1)) + d5 * sin(q(1) + q(2) + q(3)) * cos(q(0));
    J(2,0) = 0.0;
    J(3,0) = 0.0;
    J(4,0) = 0.0;
    J(5,0) = 1.0;

    J(0,1) = -cos(q(0)) * (a3 * sin(q(1) + q(2)) + a2 * sin(q(1)) - d5 * cos(q(1) + q(2) + q(3)) - d6 * sin(q(1) + q(2) + q(3)) * sin(q(4)));
    J(1,1) = -sin(q(0)) * (a3 * sin(q(1) + q(2)) + a2 * sin(q(1)) - d5 * cos(q(1) + q(2) + q(3)) - d6 * sin(q(1) + q(2) + q(3)) * sin(q(4)));
    J(2,1) = a3 * cos(q(1) + q(2)) + a2 * cos(q(1)) + d5 * (cos(q(1) + q(2)) * sin(q(3)) + sin(q(1) + q(2)) * cos(q(3))) - d6 * sin(q(4)) * (cos(q(1) + q(2)) * cos(q(3)) - sin(q(1) + q(2)) * sin(q(3)));
    J(3,1) = sin(q(0));
    J(4,1) = -cos(q(0));
    J(5,1) = 0.0;

    J(0,2) = cos(q(0)) * (d5 * cos(q(1) + q(2) + q(3)) - a3 * sin(q(1) + q(2)) + d6 * sin(q(1) + q(2) + q(3)) * sin(q(4)));
    J(1,2) = sin(q(0)) * (d5 * cos(q(1) + q(2) + q(3)) - a3 * sin(q(1) + q(2)) + d6 * sin(q(1) + q(2) + q(3)) * sin(q(4)));
    J(2,2) = a3 * cos(q(1) + q(2)) + d5 * sin(q(1) + q(2) + q(3)) - d6 * cos(q(1) + q(2) + q(3)) * sin(q(4));
    J(3,2) = sin(q(0));
    J(4,2) = -cos(q(0));
    J(5,2) = 0.0;

    J(0,3) = cos(q(0)) * (d5 * cos(q(1) + q(2) + q(3)) + d6 * sin(q(1) + q(2) + q(3)) * sin(q(4)));
    J(1,3) = sin(q(0)) * (d5 * cos(q(1) + q(2) + q(3)) + d6 * sin(q(1) + q(2) + q(3)) * sin(q(4)));
    J(2,3) = d5 * sin(q(1) + q(2) + q(3)) - d6 * cos(q(1) + q(2) + q(3)) * sin(q(4));
    J(3,3) = sin(q(0));
    J(4,3) = -cos(q(0));
    J(5,3) = 0.0;

    J(0,4) = -d6 * (sin(q(0)) * sin(q(4)) + cos(q(1) + q(2) + q(3)) * cos(q(0)) * cos(q(4)));
    J(1,4) = d6 * (cos(q(0)) * sin(q(4)) - cos(q(1) + q(2) + q(3)) * cos(q(4)) * sin(q(0)));
    J(2,4) = -d6 * sin(q(1) + q(2) + q(3)) * cos(q(4));
    J(3,4) = sin(q(1) + q(2) + q(3)) * cos(q(0));
    J(4,4) = sin(q(1) + q(2) + q(3)) * sin(q(0));
    J(5,4) = -cos(q(1) + q(2) + q(3));

    J(0,5) = 0.0;
    J(1,5) = 0.0;
    J(2,5) = 0.0;
    J(3,5) = cos(q(4)) * sin(q(0)) - cos(q(1) + q(2) + q(3)) * cos(q(0)) * sin(q(4));
    J(4,5) = -cos(q(0)) * cos(q(4)) - cos(q(1) + q(2) + q(3)) * sin(q(0)) * sin(q(4));
    J(5,5) = -sin(q(1) + q(2) + q(3)) * sin(q(4));

    // 将小于阈值的元素设为0
    const double eps = 1e-6;
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            if (std::abs(J(i,j)) < eps) {
                J(i,j) = 0.0;
            }
        }
    }

    return J;
}

moveit_msgs::msg::RobotTrajectory scale_trajectory_speed(moveit_msgs::msg::RobotTrajectory trajectory, double scale)
{
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

double trajectory_speed_calculate(moveit_msgs::msg::RobotTrajectory trajectory)
{
    Eigen::VectorXd joint_velocities(trajectory.joint_trajectory.joint_names.size());
    Eigen::VectorXd joint_positions(trajectory.joint_trajectory.joint_names.size());
    Eigen::VectorXd end_effector_velocity(trajectory.joint_trajectory.joint_names.size());
    Eigen::VectorXd end_effector_velocity_magnitude(31);  // 修改为31以匹配循环次数
    int index = 0;
    for (int i = 0; i < 31; i++)
    {
        index = trajectory.joint_trajectory.points.size() * i / 31;
        for (size_t j = 0; j < trajectory.joint_trajectory.joint_names.size(); ++j) {
            joint_velocities(j) = trajectory.joint_trajectory.points[index].velocities[j];
            joint_positions(j) = trajectory.joint_trajectory.points[index].positions[j];
        }
        end_effector_velocity = fastJacob(joint_positions) * joint_velocities;
        end_effector_velocity_magnitude(i) = sqrt(end_effector_velocity(0)*end_effector_velocity(0) + end_effector_velocity(1)*end_effector_velocity(1) + end_effector_velocity(2)*end_effector_velocity(2));
    }

    // 将速度值复制到数组中并排序
    std::vector<double> speeds(31);
    for(int i = 0; i < 31; i++) {
        speeds[i] = end_effector_velocity_magnitude(i);
        RCLCPP_INFO(LOGGER, "speed %d: %f", i, speeds[i]);
    }
    std::sort(speeds.begin(), speeds.end());
    
    return speeds[30]; // 返回中值（第个值）
}

void save_trajectory_to_file(const moveit_msgs::msg::RobotTrajectory& trajectory, const std::string& filename)
{
    std::ofstream file(filename);
    if (!file.is_open()) {
        RCLCPP_ERROR(LOGGER, "无法打开文件: %s", filename.c_str());
        return;
    }

    // 写入文件头信息
    file << "# 轨迹信息文件\n";
    file << "# 关节名称: ";
    for (size_t i = 0; i < trajectory.joint_trajectory.joint_names.size(); ++i) {
        file << trajectory.joint_trajectory.joint_names[i];
        if (i < trajectory.joint_trajectory.joint_names.size() - 1) {
            file << ", ";
        }
    }
    file << "\n";
    file << "# 轨迹点数量: " << trajectory.joint_trajectory.points.size() << "\n";
    file << "# 格式: 时间(秒), 关节位置(弧度), 关节速度(弧度/秒), 关节加速度(弧度/秒²)\n";
    file << "# 时间, ";
    for (size_t i = 0; i < trajectory.joint_trajectory.joint_names.size(); ++i) {
        file << "pos_" << trajectory.joint_trajectory.joint_names[i] << ", ";
    }
    for (size_t i = 0; i < trajectory.joint_trajectory.joint_names.size(); ++i) {
        file << "vel_" << trajectory.joint_trajectory.joint_names[i];
        if (i < trajectory.joint_trajectory.joint_names.size() - 1) {
            file << ", ";
        }
    }
    file << ", ";
    for (size_t i = 0; i < trajectory.joint_trajectory.joint_names.size(); ++i) {
        file << "acc_" << trajectory.joint_trajectory.joint_names[i];
        if (i < trajectory.joint_trajectory.joint_names.size() - 1) {
            file << ", ";
        }
    }
    file << "\n";

    // 写入轨迹数据
    for (size_t i = 0; i < trajectory.joint_trajectory.points.size(); ++i) {
        const auto& point = trajectory.joint_trajectory.points[i];
        
        // 计算时间（秒）
        double time_sec = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9;
        file << time_sec << ", ";
        
        // 写入关节位置
        for (size_t j = 0; j < point.positions.size(); ++j) {
            file << point.positions[j];
            if (j < point.positions.size() - 1) {
                file << ", ";
            }
        }
        
        // 写入关节速度
        if (!point.velocities.empty()) {
            file << ", ";
            for (size_t j = 0; j < point.velocities.size(); ++j) {
                file << point.velocities[j];
                if (j < point.velocities.size() - 1) {
                    file << ", ";
                }
            }
        }
        
        // 写入关节加速度
        if (!point.accelerations.empty()) {
            file << ", ";
            for (size_t j = 0; j < point.accelerations.size(); ++j) {
                file << point.accelerations[j];
                if (j < point.accelerations.size() - 1) {
                    file << ", ";
                }
            }
        }
        
        file << "\n";
    }
    
    file.close();
    RCLCPP_INFO(LOGGER, "轨迹信息已保存到文件: %s", filename.c_str());
}

void trajectory_to_path(moveit_msgs::msg::RobotTrajectory trajectory, moveit::core::RobotStatePtr robot_state, const moveit::core::JointModelGroup* joint_model_group)
{
	geometry_msgs::msg::PoseStamped pose_stamped;
	std::vector<double> joint_values;
	const std::string end_effector_link = "wrist_3_link";
	path.header.frame_id = "base_link";
	path.header.stamp = move_group_node->get_clock()->now();

	for (int i = 0; i < trajectory.joint_trajectory.points.size(); i++)
	{
		joint_values = trajectory.joint_trajectory.points[i].positions;
		robot_state->setJointGroupPositions(joint_model_group, joint_values);
		
		// 获取末端执行器的变换矩阵
		const Eigen::Isometry3d& transform = robot_state->getGlobalLinkTransform(end_effector_link);
		
		// 将 Eigen 变换矩阵转换为 ROS Pose 消息
		pose_stamped.pose.position.x = transform.translation().x();
		pose_stamped.pose.position.y = transform.translation().y();
		pose_stamped.pose.position.z = transform.translation().z();
		
		// 将旋转矩阵转换为四元数
		Eigen::Quaterniond q(transform.rotation());
		pose_stamped.pose.orientation.x = q.x();
		pose_stamped.pose.orientation.y = q.y();
		pose_stamped.pose.orientation.z = q.z();
		pose_stamped.pose.orientation.w = q.w();
		
		pose_stamped.header.stamp = move_group_node->get_clock()->now();
		path.poses.push_back(pose_stamped);
	}
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

class TrajectoryExecutor : public rclcpp::Node
{
public:
    TrajectoryExecutor() : Node("DIW_ui_moveit")
    {
        // 先创建发布者和订阅者
        pressure_control_pub = this->create_publisher<std_msgs::msg::Int32>("/pressure_control", 10);
        pressure_adjust_pub = this->create_publisher<std_msgs::msg::Int32>("/pressure_adjust", 10);
        start_correct_distance_pub = this->create_publisher<std_msgs::msg::Bool>("/start_correct_distance", 10);
        whether_distance_collect_pub = this->create_publisher<std_msgs::msg::Bool>("/whether_distance_collect", 10);
        // distance_platform_pub = this->create_publisher<std_msgs::msg::Float64>("/distance_platform", 10);
        start_expend_kalman_filter_pub = this->create_publisher<std_msgs::msg::Bool>("/start_expend_kalman_filter", 10);
        start_torque_correction_pub = this->create_publisher<std_msgs::msg::Bool>("/start_torque_correction", 10);
        position_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/forward_position_controller/commands", 10);

        // 创建switch_controller客户端，service控制端
        switch_controller_client = this->create_client<controller_manager_msgs::srv::SwitchController>(
            "controller_manager/switch_controller");

        // 创建关节状态订阅者
        joint_state_sub = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&TrajectoryExecutor::joint_state_callback, this, std::placeholders::_1));
        
        // 创建 move_group，使用 shared_from_this() 获取正确的节点指针
        move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){}), PLANNING_GROUP);
        
        // 创建订阅者
        diw_ui_printing_subscription = this->create_subscription<my_custom_msgs::msg::DiwUiPrint>(
            "/diw_ui_printing", 10, 
            std::bind(&TrajectoryExecutor::diw_ui_print_callback, this, std::placeholders::_1));

        pressure_control_ui_subscription = this->create_subscription<std_msgs::msg::Int32>(
            "/pressure_control_ui", 10, 
            std::bind(&TrajectoryExecutor::pressure_control_ui_callback, this, std::placeholders::_1));

        whether_trajectory_output_subscription = this->create_subscription<my_custom_msgs::msg::TrajectoryOutput>(
            "/whether_trajectory_output", 10, 
            std::bind(&TrajectoryExecutor::whether_trajectory_output_callback, this, std::placeholders::_1));

        move_tcp_subscription = this->create_subscription<geometry_msgs::msg::Pose>(
            "/move_tcp", 10, 
            std::bind(&TrajectoryExecutor::move_tcp_callback, this, std::placeholders::_1));

        test_forward_position_controller_subscription = this->create_subscription<my_custom_msgs::msg::TestForwardPositionController>(
            "/test_forward_position_controller_topic", 10, 
            std::bind(&TrajectoryExecutor::test_forward_position_controller_callback, this, std::placeholders::_1));

        // 创建定时器，以250Hz频率发布（4ms间隔）
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(4),
            std::bind(&TrajectoryExecutor::publishPositionCommand, this));
            
        // 初始化时间
        current_point_index_ = 0;
        
        // 取消定时器，直到发布轨迹信息
        timer_->cancel();    

        // 初始化其他必要的组件
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        moveit::core::RobotModelConstPtr robot_model = move_group->getRobotModel();
        RCLCPP_INFO(LOGGER, "Model frame: %s", robot_model->getModelFrame().c_str());
        moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(move_group->getRobotModel()));
        robot_state->setToDefaultValues();
        const moveit::core::JointModelGroup* joint_model_group = move_group->getRobotModel()->getJointModelGroup(PLANNING_GROUP);

        RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group->getPlanningFrame().c_str());
        RCLCPP_INFO(LOGGER, "End effector link: %s", move_group->getEndEffectorLink().c_str());
        RCLCPP_INFO(LOGGER, "Available Planning Groups:");
        std::copy(move_group->getJointModelGroupNames().begin(), move_group->getJointModelGroupNames().end(),
                    std::ostream_iterator<std::string>(std::cout, ", "));
        
    }

private:
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        last_joint_state_time = this->now();
        last_joint_state = *msg;  // Store the latest joint state
    }

    void diw_ui_print_callback(const my_custom_msgs::msg::DiwUiPrint::SharedPtr msg)
    {
        RCLCPP_INFO(LOGGER, "Received trajectory command");

        // 检查机器人状态
        try
        {
            // 检查关节状态是否是最新的
            if ((this->now() - last_joint_state_time).seconds() > 1.0)
            {
                RCLCPP_ERROR(LOGGER, "Joint state is too old. Last update was %f seconds ago", 
                    (this->now() - last_joint_state_time).seconds());
                return;
            }

            // 使用存储的关节状态信息
            if (last_joint_state.position.empty())
            {
                RCLCPP_ERROR(LOGGER, "No joint state information available");
                return;
            }

        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(LOGGER, "Error processing joint state: %s", e.what());
            return;
        }
        
        std::string file_path = msg->file_path;
        int trajectory_type = msg->trajectory_type;
        int pressure_adjust_value = msg->pressure_adjust_value;
        double printing_velocity = msg->printing_velocity;
        bool whether_start_correct_distance = msg->whether_start_correct_distance;
        bool whether_distance_collect = msg->whether_distance_collect;
        bool whether_expend_kalman_filter = msg->start_expend_kalman_filter;
        double distance_platform = msg->distance_platform;

        std::vector<geometry_msgs::msg::Pose> waypoints, waypoints_to_start, waypoints_to_end;
        waypoints = read_to_waypoints(file_path);

        geometry_msgs::msg::Pose target_pose;
        target_pose = waypoints[0];
        target_pose.position.z = target_pose.position.z + 0.01;
        waypoints_to_start.push_back(target_pose);
        waypoints_to_start.push_back(waypoints[0]);

        std_msgs::msg::Int32 msg_pressure_adjust;
        msg_pressure_adjust.data = pressure_adjust_value;
        pressure_adjust_pub->publish(msg_pressure_adjust);

        moveit_msgs::msg::RobotTrajectory trajectory_to_start, trajectory_to_start_scaled;
        const double jump_threshold = 0.0;
        const double eef_step = 0.00001; // 0.01mm
        double fraction_to_start = move_group->computeCartesianPath(waypoints_to_start, eef_step, jump_threshold, trajectory_to_start);
        trajectory_to_start_scaled = scale_trajectory_speed(trajectory_to_start, 0.1);
        move_group->execute(trajectory_to_start_scaled);

        moveit_msgs::msg::RobotTrajectory trajectory;
        double fraction = move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        
        if (fraction < 0.9)
        {
            RCLCPP_ERROR(LOGGER, "Failed to compute Cartesian path (fraction: %f)", fraction);
            return;
        }
        
        moveit_msgs::msg::RobotTrajectory trajectory_postprocess;

        if (trajectory_type == 0)
        {
            trajectory_postprocess = scale_trajectory_speed(trajectory, printing_velocity * 0.005); // 0.005对应1mm/s
        }
        else if (trajectory_type == 1)
        {
            try
            {
                // robot_trajectory::RobotTrajectory rt(move_group->getRobotModel(), move_group->getName());
                // // 创建RobotState并设置关节值
                // moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(move_group->getRobotModel()));
                // std::map<std::string, double> joint_values;
                // for (size_t i = 0; i < last_joint_state.name.size(); ++i)
                // {
                //     joint_values[last_joint_state.name[i]] = last_joint_state.position[i];
                // }
                // robot_state->setVariablePositions(joint_values);
                
                // rt.setRobotTrajectoryMsg(*robot_state, trajectory);
                // trajectory_processing::IterativeParabolicTimeParameterization iptp;
                // double rt_set_velocity = 0.00258;
                // double rt_set_acc = 0.01;
                // iptp.computeTimeStamps(rt, rt_set_velocity, rt_set_acc);
                // rt.getRobotTrajectoryMsg(trajectory);
                // moveit::planning_interface::MoveGroupInterface::Plan plan;
                // trajectory_postprocess = trajectory;
                
                trajectory_processing::TimeOptimalTrajectoryGeneration time_param(0.1, 0.02, 0.001);
                robot_trajectory::RobotTrajectory rt(move_group->getRobotModel(), move_group->getName());
                
                // 创建RobotState并设置关节值
                moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(move_group->getRobotModel()));
                std::map<std::string, double> joint_values;
                for (size_t i = 0; i < last_joint_state.name.size(); ++i)
                {
                    joint_values[last_joint_state.name[i]] = last_joint_state.position[i];
                }
                robot_state->setVariablePositions(joint_values);
                
                rt.setRobotTrajectoryMsg(*robot_state, trajectory);
                time_param.computeTimeStamps(rt, printing_velocity * 0.001 * 0.8, 0.001); // 0.02对应25mm/s
                rt.getRobotTrajectoryMsg(trajectory);
                // trajectory_postprocess = trajectory;
                
                double average_velocity = trajectory_speed_calculate(trajectory);
                RCLCPP_INFO(LOGGER, "Average velocity: %f", average_velocity);
                double velocity_scale = printing_velocity / average_velocity;
                moveit_msgs::msg::RobotTrajectory trajectory_postprocess_scaled;
                trajectory_postprocess_scaled = scale_trajectory_speed(trajectory, velocity_scale);

                trajectory_postprocess = trajectory_postprocess_scaled;

                // for (const auto& point : trajectory.joint_trajectory.points) {
                //     // 你想输出time_from_start
                //     RCLCPP_INFO(this->get_logger(), "time_from_start: %f", point.time_from_start.sec + point.time_from_start.nanosec * 1e-9);
                // }
                RCLCPP_INFO(LOGGER, "Number of points in trajectory: %zu", trajectory.joint_trajectory.points.size());
            }
            catch (const std::exception& e)
            {
                RCLCPP_ERROR(LOGGER, "Error in trajectory processing: %s", e.what());
                return;
            }
        }
        
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory_postprocess;

        if (whether_expend_kalman_filter)
        {
            auto msg_start_torque_correction = std_msgs::msg::Bool();
            msg_start_torque_correction.data = true;
            start_torque_correction_pub->publish(msg_start_torque_correction);

            std::this_thread::sleep_for(std::chrono::milliseconds(10000));

            auto msg_start_expend_kalman_filter = std_msgs::msg::Bool();
            msg_start_expend_kalman_filter.data = true;
            start_expend_kalman_filter_pub->publish(msg_start_expend_kalman_filter);

            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        
        // 发布压力控制消息
        std_msgs::msg::Int32 msg_pressure_control;
        msg_pressure_control.data = 0;  
        pressure_control_pub->publish(msg_pressure_control);

        // 发布激光测距调整信息
        if (whether_start_correct_distance)
        {
            auto msg_start_correct_distance = std_msgs::msg::Bool();
            msg_start_correct_distance.data = true;
            start_correct_distance_pub->publish(msg_start_correct_distance);
        }

        if (whether_distance_collect)
        {
            auto msg_whether_distance_collect = std_msgs::msg::Bool();
            msg_whether_distance_collect.data = true;
            whether_distance_collect_pub->publish(msg_whether_distance_collect);
        }    

        // 执行轨迹
        try
        {
            auto start_time = std::chrono::high_resolution_clock::now();
            move_group->execute(plan);
            auto end_time = std::chrono::high_resolution_clock::now();

            // 计算执行时间
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
            RCLCPP_INFO(LOGGER, "Trajectory execution time: %ld ms", duration);
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(LOGGER, "Error executing trajectory: %s", e.what());
            return;
        }



        // 发布压力控制消息
        msg_pressure_control.data = 1;  
        pressure_control_pub->publish(msg_pressure_control);

        // 发布激光测距调整信息
        if (whether_start_correct_distance)
        {
            auto msg_start_correct_distance = std_msgs::msg::Bool();
            msg_start_correct_distance.data = false;
            start_correct_distance_pub->publish(msg_start_correct_distance);
        }

        if (whether_expend_kalman_filter)
        {
            auto msg_start_expend_kalman_filter = std_msgs::msg::Bool();
            msg_start_expend_kalman_filter.data = false;
            start_expend_kalman_filter_pub->publish(msg_start_expend_kalman_filter);

            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }

        if (whether_distance_collect)
        {
            auto msg_whether_distance_collect = std_msgs::msg::Bool();
            msg_whether_distance_collect.data = false;
            whether_distance_collect_pub->publish(msg_whether_distance_collect);
        }

        geometry_msgs::msg::Pose target_pose_to_end;
        target_pose_to_end = waypoints[waypoints.size() - 1];
        target_pose_to_end.position.z = target_pose_to_end.position.z + 0.01;
        waypoints_to_end.push_back(target_pose_to_end);
        moveit_msgs::msg::RobotTrajectory trajectory_to_end, trajectory_to_end_scaled;
        double fraction_to_end = move_group->computeCartesianPath(waypoints_to_end, eef_step, jump_threshold, trajectory_to_end);
        trajectory_to_end_scaled = scale_trajectory_speed(trajectory_to_end, 0.1);
        move_group->execute(trajectory_to_end_scaled);
    }

    void pressure_control_ui_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        auto msg_pressure_control = std_msgs::msg::Int32();
        msg_pressure_control.data = msg->data;
        pressure_control_pub->publish(msg_pressure_control);
    }

    void whether_trajectory_output_callback(const my_custom_msgs::msg::TrajectoryOutput::SharedPtr msg)
    {
        std::string file_path = msg->file_path;
        bool whether_trajectory_output = msg->whether_trajectory_output;
        double planning_velocity = msg->planning_velocity;
        double control_period = msg->control_period;

        if (whether_trajectory_output)
        {
            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints = read_to_waypoints(file_path);

            const double jump_threshold = 0.0;
            const double eef_step = 0.00001; // 0.01mm
            moveit_msgs::msg::RobotTrajectory trajectory, trajectory_postprocess;

            double fraction = move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
            
            if (fraction < 0.9)
            {
                RCLCPP_ERROR(LOGGER, "Failed to compute Cartesian path (fraction: %f)", fraction);
                return;
            }

            try
            {     
                trajectory_processing::TimeOptimalTrajectoryGeneration time_param(0.00002, control_period, 0.00002);
                robot_trajectory::RobotTrajectory rt(move_group->getRobotModel(), move_group->getName());
                
                // 创建RobotState并设置关节值
                moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(move_group->getRobotModel()));
                std::map<std::string, double> joint_values;
                for (size_t i = 0; i < last_joint_state.name.size(); ++i)
                {
                    joint_values[last_joint_state.name[i]] = last_joint_state.position[i];
                }
                robot_state->setVariablePositions(joint_values);
                
                rt.setRobotTrajectoryMsg(*robot_state, trajectory);
                time_param.computeTimeStamps(rt, planning_velocity * 0.001 * 0.8, 0.001); // 0.02对应25mm/s，这个缩放倍率就是限制关节速度的，与TCP的绝对速度不同
                rt.getRobotTrajectoryMsg(trajectory);
                
                // double average_velocity = trajectory_speed_calculate(trajectory);
                // double velocity_scale = planning_velocity / average_velocity;
                // moveit_msgs::msg::RobotTrajectory trajectory_postprocess_scaled;
                // trajectory_postprocess_scaled = scale_trajectory_speed(trajectory, velocity_scale);
                // average_velocity = trajectory_speed_calculate(trajectory_postprocess_scaled);
                // RCLCPP_INFO(LOGGER, "Average velocity: %f", average_velocity);

                // 保存轨迹信息到文件
                auto now = std::chrono::system_clock::now();
                auto time_t = std::chrono::system_clock::to_time_t(now);
                std::stringstream ss;
                ss << "/home/xht/moveit2_workspace/src/ur3_moveit/config/trajectory_output/trajectory_postprocess_" 
                   << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S") << ".txt";
                save_trajectory_to_file(trajectory, ss.str());

                // for (const auto& point : trajectory.joint_trajectory.points) {
                //     // 你想输出time_from_start
                //     RCLCPP_INFO(this->get_logger(), "time_from_start: %f", point.time_from_start.sec + point.time_from_start.nanosec * 1e-9);
                // }
                RCLCPP_INFO(LOGGER, "Number of points in trajectory: %zu", trajectory.joint_trajectory.points.size());
            }
            catch (const std::exception& e)
            {
                RCLCPP_ERROR(LOGGER, "Error in trajectory processing: %s", e.what());
                return;
            }

            
            
        }
    }

    void move_tcp_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        std::vector<geometry_msgs::msg::Pose> waypoints_move_tcp;
        waypoints_move_tcp.push_back(*msg);
        moveit_msgs::msg::RobotTrajectory trajectory_move_tcp, trajectory_move_tcp_scaled;

        const double jump_threshold = 0.0;
        const double eef_step = 0.00001; // 0.01mm
        double fraction_to_start = move_group->computeCartesianPath(waypoints_move_tcp, eef_step, jump_threshold, trajectory_move_tcp);
        trajectory_move_tcp_scaled = scale_trajectory_speed(trajectory_move_tcp, 0.1);
        move_group->execute(trajectory_move_tcp_scaled);
    }

    void test_forward_position_controller_callback(const my_custom_msgs::msg::TestForwardPositionController::SharedPtr msg)
    {
        double printing_velocity = msg->printing_velocity;
        double control_period = msg->control_period;

        std::vector<geometry_msgs::msg::Pose> waypoints_test_forward_position_controller_start, waypoints_test_forward_position_controller;

        waypoints_test_forward_position_controller = read_to_waypoints(msg->file_path);
        waypoints_test_forward_position_controller_start.push_back(waypoints_test_forward_position_controller[0]);

        // 第一段回到打印原点
        const double jump_threshold = 0.0;
        const double eef_step = 0.00001; // 0.01mm
        moveit_msgs::msg::RobotTrajectory trajectory_test_forward_position_controller_start, trajectory_test_forward_position_controller_start_scaled;
        double fraction_to_start = move_group->computeCartesianPath(waypoints_test_forward_position_controller_start, eef_step, jump_threshold, trajectory_test_forward_position_controller_start);
        trajectory_test_forward_position_controller_start_scaled = scale_trajectory_speed(trajectory_test_forward_position_controller_start, 0.1);
        move_group->execute(trajectory_test_forward_position_controller_start_scaled);

        //第二段计算轨迹
        double fraction = move_group->computeCartesianPath(waypoints_test_forward_position_controller, eef_step, jump_threshold, trajectory_test_forward_position_controller);
        
        if (fraction < 0.9)
            {
                RCLCPP_ERROR(LOGGER, "Failed to compute Cartesian path (fraction: %f)", fraction);
                return;
            }

            try
            {     
                trajectory_processing::TimeOptimalTrajectoryGeneration time_param(0.00002, control_period, 0.00002);
                robot_trajectory::RobotTrajectory rt(move_group->getRobotModel(), move_group->getName());
                
                // 创建RobotState并设置关节值
                moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(move_group->getRobotModel()));
                std::map<std::string, double> joint_values;
                for (size_t i = 0; i < last_joint_state.name.size(); ++i)
                {
                    joint_values[last_joint_state.name[i]] = last_joint_state.position[i];
                }
                robot_state->setVariablePositions(joint_values);
                
                rt.setRobotTrajectoryMsg(*robot_state, trajectory_test_forward_position_controller);
                time_param.computeTimeStamps(rt, printing_velocity * 0.001 * 0.8, 0.001); // 0.02对应25mm/s，这个缩放倍率就是限制关节速度的，与TCP的绝对速度不同
                rt.getRobotTrajectoryMsg(trajectory_test_forward_position_controller);

                // 保存轨迹信息到文件
                auto now = std::chrono::system_clock::now();
                auto time_t = std::chrono::system_clock::to_time_t(now);
                std::stringstream ss;
                ss << "/home/xht/moveit2_workspace/src/ur3_moveit/config/test_forward_position_controller_trajectory/trajectory_postprocess_" 
                   << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S") << ".txt";
                save_trajectory_to_file(trajectory_test_forward_position_controller, ss.str());

                RCLCPP_INFO(LOGGER, "Number of points in trajectory: %zu", trajectory_test_forward_position_controller.joint_trajectory.points.size());
            }
            catch (const std::exception& e)
            {
                RCLCPP_ERROR(LOGGER, "Error in trajectory processing: %s", e.what());
                return;
            }

        if (!switch_controller_client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(LOGGER, "Service not available");
            return;
        }

        publishswitchcontrollercmds("forward_position_controller", "scaled_joint_trajectory_controller");

        timer_->reset();
    }

    void publishPositionCommand()
    {
        if (trajectory_test_forward_position_controller.joint_trajectory.points.empty()) {
            RCLCPP_ERROR(LOGGER, "Trajectory is empty!");
            timer_->cancel();
            return;
        }

        if (current_point_index_ < trajectory_test_forward_position_controller.joint_trajectory.points.size()) {
            // 发布位置指令
            auto position_msg = std_msgs::msg::Float64MultiArray();
            position_msg.data = trajectory_test_forward_position_controller.joint_trajectory.points[current_point_index_].positions;
            position_publisher_->publish(position_msg);
            current_point_index_++;
        }else{
            timer_->cancel();
            current_point_index_ = 0;
            publishswitchcontrollercmds("scaled_joint_trajectory_controller", "forward_position_controller");
            RCLCPP_INFO(LOGGER, "Trajectory execution completed");
        }
        
    }

    void publishswitchcontrollercmds(std::string controller_start_name, std::string controller_stop_name)
    {
        if (!switch_controller_client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(LOGGER, "Service not available");
            return;
        }

        auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
        request->start_controllers = {controller_start_name};
        request->stop_controllers = {controller_stop_name};
        request->strictness = request->STRICT;

        auto future_result = switch_controller_client->async_send_request(
            request,
            [this, controller_start_name, controller_stop_name](rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future) {
                if (future.get()->ok) {
                    RCLCPP_INFO(LOGGER, "Controller switch successful, controller_start_name: %s, controller_stop_name: %s", controller_start_name.c_str(), controller_stop_name.c_str());
                } else {
                    RCLCPP_ERROR(LOGGER, "Controller switch failed, controller_start_name: %s, controller_stop_name: %s", controller_start_name.c_str(), controller_stop_name.c_str());
                    return;
                }
            });
    }


    rclcpp::Subscription<my_custom_msgs::msg::DiwUiPrint>::SharedPtr diw_ui_printing_subscription;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr pressure_control_ui_subscription;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub;
    rclcpp::Subscription<my_custom_msgs::msg::TrajectoryOutput>::SharedPtr whether_trajectory_output_subscription;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr move_tcp_subscription;
    rclcpp::Subscription<my_custom_msgs::msg::TestForwardPositionController>::SharedPtr test_forward_position_controller_subscription;
    
    rclcpp::Time last_joint_state_time;
    sensor_msgs::msg::JointState last_joint_state;  // Store the latest joint state
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<TrajectoryExecutor>();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}