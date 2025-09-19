#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <cmath>
#include <vector>
#include <fstream>
#include <string>
#include <chrono>
#include <iomanip>
#include <filesystem>

#include <std_msgs/msg/int32.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class ThetaDataCollectSubscriber : public rclcpp::Node
{
public:
    ThetaDataCollectSubscriber()
        : Node("theta_data_collect")  // 设置节点名称
    {
        // 声明并获取数据存储路径参数
        this->declare_parameter("data_directory", "/home/xht/moveit2_workspace/src/ur3_moveit/config/theta_data_collect");
        data_directory = this->get_parameter("data_directory").as_string();
        
        // 确保数据目录存在
        std::filesystem::create_directories(data_directory);

        // 创建订阅者，订阅 "/realtime_distance_left" 话题
        realtime_theta_data_sub = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,  // 话题名称和队列大小
            std::bind(&ThetaDataCollectSubscriber::realtime_theta_data_callback, this, std::placeholders::_1));

        theta_data_number_sub = this->create_subscription<std_msgs::msg::Int32>(
            "/theta_data_number", 10,
            std::bind(&ThetaDataCollectSubscriber::theta_data_number_callback, this, std::placeholders::_1));

        std::chrono::system_clock::time_point start_time;
        
    }

    ~ThetaDataCollectSubscriber() {
        if (output_file.is_open()) {
            output_file.close();
        }
    }

private:
    // 生成带时间戳的文件名
    std::string generate_filename() {
        auto now = std::chrono::system_clock::now();
        auto now_time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << data_directory << "/theta_data_" << std::put_time(std::localtime(&now_time_t), "%Y%m%d_%H%M%S") << ".txt";
        return ss.str();
    }

    // 回调函数：处理 "/joint_states" 话题的消息
    void realtime_theta_data_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        
        if (data_number <= theta_data_number && start_theta_data_collect == true)
        {
            auto now = std::chrono::system_clock::now();
            // 计算与开始时间的差值（秒）
            auto duration = std::chrono::duration_cast<std::chrono::duration<double>>(now - start_time);

            // 将数据写入文件
            if (output_file.is_open()) {
                output_file << std::fixed << std::setprecision(16) << duration.count()
                           << " theta " << msg->position[0] << " " << msg->position[1] << " " << msg->position[2] << " " << msg->position[3] << " " << msg->position[4] << " " << msg->position[5] << " velocity " << msg->velocity[0] << " " << msg->velocity[1] << " " << msg->velocity[2] << " " << msg->velocity[3] << " " << msg->velocity[4] << " " << msg->velocity[5] << " effort " << msg->effort[0] << " " << msg->effort[1] << " " << msg->effort[2] << " " << msg->effort[3] << " " << msg->effort[4] << " " << msg->effort[5] <<  std::endl;
            }

            data_number++;
 
        }else if (data_number > theta_data_number)
        {
            if (output_file.is_open()) {
                output_file.close();
                RCLCPP_INFO(this->get_logger(), "停止记录数据");
            }
            data_number = 0;
            start_theta_data_collect = false;
        }
        
    }

    // 回调函数：处理 "/theta_data_number" 话题的消息  
    void theta_data_number_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        
        theta_data_number = msg->data;
        start_theta_data_collect = true;
        auto now = std::chrono::system_clock::now();
        if (start_theta_data_collect == true)
        {
            start_time = now;
            data_number = 0;
            // 生成带时间戳的文件名并打开文件
            std::string filename = generate_filename();
            output_file.open(filename, std::ios::app);
            if (!output_file.is_open()) {
                RCLCPP_ERROR(this->get_logger(), "无法打开文件进行写入: %s", filename.c_str());
            } else {
                RCLCPP_INFO(this->get_logger(), "开始记录数据到文件: %s", filename.c_str());
            }
        }else {
            // 关闭文件
            if (output_file.is_open()) {
                output_file.close();
                RCLCPP_INFO(this->get_logger(), "停止记录数据");
            }
        }

    } 

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr realtime_theta_data_sub;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr theta_data_number_sub;
    std::ofstream output_file;
    bool start_theta_data_collect = false;
    std::chrono::system_clock::time_point start_time;
    std::string data_directory;  // 数据存储目录
    int data_number = 0;
    int theta_data_number = 0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);  // 初始化 ROS 2
    auto node = std::make_shared<ThetaDataCollectSubscriber>();  // 创建订阅者节点
    rclcpp::spin(node);  // 保持节点运行，监听消息
    rclcpp::shutdown();  // 关闭 ROS 2
    return 0;
}