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
#include <std_msgs/msg/bool.hpp>

class DistanceCollectSubscriber : public rclcpp::Node
{
public:
    DistanceCollectSubscriber()
        : Node("distance_collect")  // 设置节点名称
    {
        // 声明并获取数据存储路径参数
        this->declare_parameter("data_directory", "/home/xht/moveit2_workspace/src/ur3_moveit/config/distance_collect");
        data_directory = this->get_parameter("data_directory").as_string();
        
        // 确保数据目录存在
        std::filesystem::create_directories(data_directory);

        // 创建订阅者，订阅 "/realtime_distance_left" 话题
        realtime_distance_left_sub = this->create_subscription<std_msgs::msg::Int32>(
            "/realtime_distance_left", 10,  // 话题名称和队列大小
            std::bind(&DistanceCollectSubscriber::realtime_distance_left_callback, this, std::placeholders::_1));

        whether_distance_collect_sub = this->create_subscription<std_msgs::msg::Bool>(
            "/whether_distance_collect", 10,
            std::bind(&DistanceCollectSubscriber::whether_distance_collect_callback, this, std::placeholders::_1));

        std::chrono::system_clock::time_point start_time;
        
    }

    ~DistanceCollectSubscriber() {
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
        ss << data_directory << "/distance_data_" << std::put_time(std::localtime(&now_time_t), "%Y%m%d_%H%M%S") << ".txt";
        return ss.str();
    }

    // 回调函数：处理 "/realtime_distance_left" 话题的消息
    void realtime_distance_left_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        
        if (start_distance_collect == true)
        {
            auto now = std::chrono::system_clock::now();
            // 计算与开始时间的差值（秒）
            auto duration = std::chrono::duration_cast<std::chrono::duration<double>>(now - start_time);

            // 将数据写入文件
            if (output_file.is_open()) {
                output_file << std::fixed << std::setprecision(3) << duration.count()
                           << " " << msg->data << std::endl;
            }

        }
        
    }

    // 回调函数：处理 "/whether_distance_collect" 话题的消息  
    void whether_distance_collect_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        
        start_distance_collect = msg->data;
        auto now = std::chrono::system_clock::now();
        if (start_distance_collect == true)
        {
            start_time = now;
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

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr realtime_distance_left_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr whether_distance_collect_sub;
    std::ofstream output_file;
    bool start_distance_collect = false;
    std::chrono::system_clock::time_point start_time;
    std::string data_directory;  // 数据存储目录
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);  // 初始化 ROS 2
    auto node = std::make_shared<DistanceCollectSubscriber>();  // 创建订阅者节点
    rclcpp::spin(node);  // 保持节点运行，监听消息
    rclcpp::shutdown();  // 关闭 ROS 2
    return 0;
}