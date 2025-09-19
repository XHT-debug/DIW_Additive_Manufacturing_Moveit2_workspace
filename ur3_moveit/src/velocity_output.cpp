#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <cmath>
#include <vector>

#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/int32.hpp>

rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_output_pub;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr odometer_output_pub;
rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pressure_active_adjust_pub;

double odometer = 0.0;

double last_x = 100.0, last_y = 100.0, last_z = 100.0;

double average_velocity = 0.001; // 1.0mm/s 速度
double average_pressure = 300; // 0.2MPa or 2.0bar

int start_pressure_active_adjust_mode = 0;
double velocity_1 = 0.0, velocity_2 = 0.0, velocity_3 = 0.0, velocity_4 = 0.0, velocity_5 = 0.0, velocity_1_5 = 0.0;

class VelocityOutputSubscriber : public rclcpp::Node
{
public:
    VelocityOutputSubscriber()
        : Node("velocity_output")  // 设置节点名称
    {
        // 创建订阅者，订阅 "/velocity_output" 话题
        tcp_velocity_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/tcp_velocity", 10,  // 话题名称和队列大小
            std::bind(&VelocityOutputSubscriber::tcp_velocity_callback, this, std::placeholders::_1));

        tcp_pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/tcp_pose_broadcaster/pose", 10,  // 话题名称和队列大小
            std::bind(&VelocityOutputSubscriber::tcp_pose_callback, this, std::placeholders::_1));

        pressure_adjust_start_sub = this->create_subscription<std_msgs::msg::Int32>(
            "/pressure_control_close", 10,  // 话题名称和队列大小
            std::bind(&VelocityOutputSubscriber::pressure_control_callback, this, std::placeholders::_1)); // 想要实现最原始的气压速度自适应算法，需要把后缀_close删除

        velocity_output_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>("/velocity_curve", 10);

        odometer_output_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/odometer_curve", 10);

        pressure_active_adjust_pub = this->create_publisher<std_msgs::msg::Int32>("/pressure_adjust", 10);

    }

private:
    // 回调函数：处理 "/tcp_velocity" 话题的消息
    void tcp_velocity_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        
        // 确保消息中包含足够的数据
        if (msg->data.size() < 8) {
            RCLCPP_ERROR(this->get_logger(), "tcp_velocity message does not contain enough data.");
            return;
        }

        auto msg_velocity_output = geometry_msgs::msg::TwistStamped();

        auto msg_pressure_active_adjust = std_msgs::msg::Int32();

        // 传输/joint_state的时间戳
        
        msg_velocity_output.header.stamp.sec = static_cast<int>(msg->data[6]);
        msg_velocity_output.header.stamp.nanosec = static_cast<int>(msg->data[7]);
        msg_velocity_output.header.frame_id = "base";
        
        // 计算当前速度,m/s为速度单位
        double velocity_magnitude = sqrt(msg->data[0] * msg->data[0] + msg->data[1] * msg->data[1] + msg->data[2] * msg->data[2]);

        msg_velocity_output.twist.linear.x = msg->data[0];
        msg_velocity_output.twist.linear.y = msg->data[1];
        msg_velocity_output.twist.linear.z = msg->data[2];

        // 设置角速度
        msg_velocity_output.twist.angular.x = velocity_magnitude;
        msg_velocity_output.twist.angular.y = 0.0;
        msg_velocity_output.twist.angular.z = 0.0;

        velocity_1 = velocity_2;
        velocity_2 = velocity_3;
        velocity_3 = velocity_4;
        velocity_4 = velocity_5;
        velocity_5 = velocity_magnitude;
        velocity_1_5 = (velocity_1 + velocity_2 + velocity_3 + velocity_4 + velocity_5) / 5;
        
        if (velocity_1_5 < 1.2 * average_velocity && velocity_1_5 > 0.8 * average_velocity && start_pressure_active_adjust_mode == 1)
        {
            msg_pressure_active_adjust.data = static_cast<int32_t>(average_pressure + (velocity_1_5 - average_velocity) * average_pressure * 0.5 / average_velocity);
            pressure_active_adjust_pub->publish(msg_pressure_active_adjust);
            RCLCPP_INFO(this->get_logger(), "Publishing pressure adjustment: %d", msg_pressure_active_adjust.data);
        }
        
        velocity_output_pub->publish(msg_velocity_output);
    
    }

     // 回调函数：处理 "/tcp_pose" 话题的消息
     void tcp_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // 确保消息中包含足够的关节数据

        auto msg_odometer_output = geometry_msgs::msg::PoseStamped();

        // 传输/joint_state的时间戳
        
        msg_odometer_output.header.stamp.sec = msg->header.stamp.sec;
        msg_odometer_output.header.stamp.nanosec = msg->header.stamp.nanosec;
        msg_odometer_output.header.frame_id = "base";
        
        // 设置TCP位置
        msg_odometer_output.pose.position.x = msg->pose.position.x;
        msg_odometer_output.pose.position.y = msg->pose.position.y;
        msg_odometer_output.pose.position.z = (msg->pose.position.z * 1000 - 269.4) * 1000;


        // 里程计计算
        if (last_x == 100.0 && last_y == 100.0 && last_z == 100.0)
        {
            odometer = 0.0;
            last_x = msg_odometer_output.pose.position.x;
            last_y = msg_odometer_output.pose.position.y;
            last_z = msg_odometer_output.pose.position.z;

        }
        else
        {
            odometer = odometer + sqrt((msg_odometer_output.pose.position.x - last_x) * (msg_odometer_output.pose.position.x - last_x) + (msg_odometer_output.pose.position.y - last_y) * (msg_odometer_output.pose.position.y - last_y) + (msg_odometer_output.pose.position.z - last_z) * (msg_odometer_output.pose.position.z - last_z));
            last_x = msg_odometer_output.pose.position.x;
            last_y = msg_odometer_output.pose.position.y;
            last_z = msg_odometer_output.pose.position.z;
        }
        

        // 设置里程计
        msg_odometer_output.pose.orientation.x = odometer;
        msg_odometer_output.pose.orientation.y = 0.0;
        msg_odometer_output.pose.orientation.z = 0.0;
        msg_odometer_output.pose.orientation.w = 0.0;

        odometer_output_pub->publish(msg_odometer_output);
    
    }

     // 回调函数：处理 "/pressure_control" 话题的消息
     void pressure_control_callback(const std_msgs::msg::Int32::SharedPtr msg) {

        if (msg->data == 0)
        {
            start_pressure_active_adjust_mode = 1;
        }
        else if (msg->data == 1)
        {
            start_pressure_active_adjust_mode = 0;
        }

    }


    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr tcp_velocity_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr tcp_pose_sub;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr pressure_adjust_start_sub;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);  // 初始化 ROS 2
    auto node = std::make_shared<VelocityOutputSubscriber>();  // 创建订阅者节点
    rclcpp::spin(node);  // 保持节点运行，监听消息
    rclcpp::shutdown();  // 关闭 ROS 2
    return 0;
}