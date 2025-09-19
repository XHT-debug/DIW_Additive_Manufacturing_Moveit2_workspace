#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <cmath>
#include <vector>

#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/int32.hpp>


rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pressure_control_pub;

int start_test_delay_mode = 0;


class PressureControlSubscriber : public rclcpp::Node
{
public:
    PressureControlSubscriber()
        : Node("test_delay")  // 设置节点名称
    {
        // 创建订阅者，订阅 "/pressure_control" 话题
        pressure_control_sub = this->create_subscription<std_msgs::msg::Int32>(
            "/pressure_control", 10,  // 话题名称和队列大小
            std::bind(&PressureControlSubscriber::pressure_control_callback, this, std::placeholders::_1));


        pressure_control_pub = this->create_publisher<std_msgs::msg::Int32>("/pressure_control", 10);

    }

private:
    // 回调函数：处理 "/pressure_control" 话题的消息
    void pressure_control_callback(const std_msgs::msg::Int32::SharedPtr msg) {

        auto msg_pub = std_msgs::msg::Int32();

        auto start_time = std::chrono::high_resolution_clock::now();

        auto end_time = std::chrono::high_resolution_clock::now();

        // 计算执行时间
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

        // 输出执行时间
        std::cout << "Trajectory execution time: " << duration << " ms" << std::endl;
        if (msg->data == 0)
        {
            start_test_delay_mode = 1;
            msg_pub.data = 1;
            auto start_time = std::chrono::high_resolution_clock::now();
            pressure_control_pub->publish(msg_pub);
        }
        else if (msg->data == 1 && start_test_delay_mode == 1)
        {
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
            msg_pub.data = 0;
            pressure_control_pub->publish(msg_pub);
        }

    }

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr pressure_control_sub;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);  // 初始化 ROS 2
    auto node = std::make_shared<PressureControlSubscriber>();  // 创建订阅者节点
    rclcpp::spin(node);  // 保持节点运行，监听消息
    rclcpp::shutdown();  // 关闭 ROS 2
    return 0;
}