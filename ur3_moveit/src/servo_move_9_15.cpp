#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
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

#include "my_custom_msgs/msg/twist_time.hpp"
#include <controller_manager_msgs/srv/switch_controller.hpp>

#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("node name: servo_move");

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
  	rclcpp::NodeOptions node_options;
	node_options.automatically_declare_parameters_from_overrides(true);
	auto move_group_node = rclcpp::Node::make_shared("ur3_moveit", node_options);

	rclcpp::executors::SingleThreadedExecutor executor;

	// auto pressure_control_pub = move_group_node->create_publisher<std_msgs::msg::Int32>("/pressure_control", 10);
	// auto pressure_adjust_pub = move_group_node->create_publisher<std_msgs::msg::Int32>("/pressure_adjust", 10);
	auto servo_twist_time_pub = move_group_node->create_publisher<my_custom_msgs::msg::TwistTime>("/servo_ur_twist_cmds", 10);
	auto switch_move_mode_pub = move_group_node->create_publisher<std_msgs::msg::Bool>("/switch_move_mode", 10);
	
	auto switch_controller_client = move_group_node->create_client<controller_manager_msgs::srv::SwitchController>("controller_manager/switch_controller");
	
	// msg_pressure_control.data = 0;  
	// pressure_control_pub->publish(msg_pressure_control);

	// std::this_thread::sleep_for(std::chrono::seconds(2));

	// msg_pressure_control.data = 1;  
	// pressure_control_pub->publish(msg_pressure_control);

	// // std_msgs::msg::Bool switch_move_mode_msg;
	// switch_move_mode_msg.data = false; 
	// switch_move_mode_pub->publish(switch_move_mode_msg);



	// std_msgs::msg::Bool switch_move_mode_msg;
	// switch_move_mode_msg.data = true; 
	// switch_move_mode_pub->publish(switch_move_mode_msg);

	// std::this_thread::sleep_for(std::chrono::seconds(1));
	
	// msg_pressure_control.data = 0;  
	// pressure_control_pub->publish(msg_pressure_control);

	my_custom_msgs::msg::TwistTime twist_time;
	twist_time.twist.header.frame_id = "base_link";
	twist_time.twist.twist.linear.y = 0.018; //0.03对应速度0.0025mm/s
	twist_time.twist.twist.linear.z = -0.0; //0.00295
	twist_time.time = 1.0;
	servo_twist_time_pub->publish(twist_time);

	std::this_thread::sleep_for(std::chrono::seconds(1));
	
	rclcpp::shutdown();
	return 0;
}


