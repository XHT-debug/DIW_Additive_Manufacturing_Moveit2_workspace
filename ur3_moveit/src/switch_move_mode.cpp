#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <cmath>
#include <vector>

#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("switch_move_mode");
rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_client;

class SwitchMoveModeSubscriber : public rclcpp::Node
{
public:
    SwitchMoveModeSubscriber()
        : Node("switch_move_mode")
    {
        switch_move_mode_sub = this->create_subscription<std_msgs::msg::Bool>(
            "/switch_move_mode", 10,
            std::bind(&SwitchMoveModeSubscriber::switch_move_mode_callback, this, std::placeholders::_1));

        switch_controller_client = this->create_client<controller_manager_msgs::srv::SwitchController>(
            "controller_manager/switch_controller");
    }

private:
    void switch_move_mode_callback(const std_msgs::msg::Bool::SharedPtr msg) 
    {
        if (!switch_controller_client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(LOGGER, "Service not available");
            return;
        }

        auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
        if (msg->data) {
            request->start_controllers = {"forward_position_controller"};
            request->stop_controllers = {"scaled_joint_trajectory_controller"};
        } else {
            request->start_controllers = {"scaled_joint_trajectory_controller"};
            request->stop_controllers = {"forward_position_controller"};
        }
        request->strictness = request->STRICT;

        // 异步发送请求并使用lambda处理响应
        auto future_result = switch_controller_client->async_send_request(
            request,
            [this](rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future) {
                if (future.get()->ok) {
                    RCLCPP_INFO(LOGGER, "Controller switch successful");
                } else {
                    RCLCPP_ERROR(LOGGER, "Controller switch failed");
                }
            });
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr switch_move_mode_sub;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SwitchMoveModeSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}