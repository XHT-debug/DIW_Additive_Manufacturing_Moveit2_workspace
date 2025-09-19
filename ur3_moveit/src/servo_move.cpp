#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <chrono>
#include <thread>
#include "my_custom_msgs/msg/twist_time.hpp"
#include <controller_manager_msgs/srv/switch_controller.hpp>

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


