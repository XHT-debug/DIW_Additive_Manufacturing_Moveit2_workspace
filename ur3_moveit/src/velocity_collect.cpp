#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <cmath>
#include <fstream>
#include <mutex>
#include <string>

#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>


double average_velocity = 0.0012; // 1.0mm/s 速度
double average_pressure = 300; // 0.3MPa or 3.0bar

int start_velocity_collect_mode = 0;
int start_sec = -1;
int start_nanosec = -1;
int if_in_start = 1;
int sec, nanosec;
double scale = 0.5; //气压速度间的调整缩放因子
double lower_limit = 0.8, upper_limit = 1.2; //气压尖峰阈值处理范围
std::mutex file_mutex;
std::ofstream pressure_file;
std::string file_path;
int pressure_adjust_value = 0;

std::string get_timestamped_filename() {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << "pressure_data_" << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S") << ".txt";
    return ss.str();
}

void close_pressure_file() {
    std::lock_guard<std::mutex> lock(file_mutex);
    if (pressure_file.is_open()) {
        pressure_file.close();
    }
}

class VelocityCollectSubscriber : public rclcpp::Node
{
public:
    VelocityCollectSubscriber()
        : Node("velocity_collect")  // 设置节点名称
    {
        // 创建订阅者，订阅 "/velocity_output" 话题
        tcp_velocity_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/tcp_velocity", 10,  // 话题名称和队列大小
            std::bind(&VelocityCollectSubscriber::tcp_velocity_callback, this, std::placeholders::_1));

        velocity_collect_switch_sub = this->create_subscription<std_msgs::msg::Int32>(
            "/velocity_collect_switch", 10,  // 话题名称和队列大小
            std::bind(&VelocityCollectSubscriber::velocity_collect_switch_callback, this, std::placeholders::_1));

    }

private:
    // 回调函数：处理 "/tcp_velocity" 话题的消息
    void tcp_velocity_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        
        // 检查收集速度模式是否开启
        if (start_velocity_collect_mode == 1)
        {
            // 确保消息中包含足够的数据
            if (msg->data.size() < 8) {
                RCLCPP_ERROR(this->get_logger(), "tcp_velocity message does not contain enough data.");
                return;
            }    
            
            sec = static_cast<int>(msg->data[6]);
            nanosec = static_cast<int>(msg->data[7]);
            
            if (start_sec == -1 && start_nanosec == -1)
            {
                start_sec = sec;
                start_nanosec = nanosec;
                std::lock_guard<std::mutex> lock(file_mutex);
                if (!pressure_file.is_open()) {
                    file_path = "/home/xht/moveit2_workspace/src/ur3_moveit/config/pressure_adjust/" + get_timestamped_filename();
                    pressure_file.open(file_path, std::ios::out); // 创建带时间戳的新文件
                    if (!pressure_file.is_open()) {
                        RCLCPP_ERROR(rclcpp::get_logger("velocity_collect"), "Failed to open velocity data file");
                        return;
                    }
                }
            }
            
            double now_time = (sec - start_sec) + (nanosec - start_nanosec) * 1e-9;
            double now_velocity = sqrt(msg->data[0] * msg->data[0] + msg->data[1] * msg->data[1] + msg->data[2] * msg->data[2]);
            
            if(now_velocity < lower_limit * average_velocity && if_in_start == 1)
            {
                pressure_adjust_value = static_cast<int>(average_pressure + (now_velocity - average_velocity) * average_pressure * scale / average_velocity);
                pressure_file << now_time << " " << pressure_adjust_value << std::endl;
                // 检查文件写入是否成功
                if (!pressure_file.good()) {
                    RCLCPP_ERROR(rclcpp::get_logger("velocity_collect"), "Failed to write to velocity data file at %s", file_path.c_str());
                }

            }else if(now_velocity > lower_limit * average_velocity && if_in_start == 1)
            {
                if_in_start = 0;
            }
            
            
            if (now_velocity < upper_limit * average_velocity && now_velocity > lower_limit * average_velocity)
            {
                pressure_adjust_value = static_cast<int>(average_pressure + (now_velocity - average_velocity) * average_pressure * scale / average_velocity);
                pressure_file << now_time << " " << pressure_adjust_value << std::endl;
                // 检查文件写入是否成功
                if (!pressure_file.good()) {
                    RCLCPP_ERROR(rclcpp::get_logger("velocity_collect"), "Failed to write to velocity data file at %s", file_path.c_str());
                }
            }


        }
        
    }

     // 回调函数：处理 "/pressure_control" 话题的消息
     void velocity_collect_switch_callback(const std_msgs::msg::Int32::SharedPtr msg) {

        if (msg->data == 1)
        {
            start_velocity_collect_mode = 1;
            if_in_start = 1;
            RCLCPP_INFO(rclcpp::get_logger("velocity_collect"), "Pressure active adjust mode started.");
        }
        else if (msg->data == 0)
        {
            
            if (start_velocity_collect_mode == 1)
            {
                close_pressure_file();
                RCLCPP_INFO(rclcpp::get_logger("velocity_collect"), "Pressure active adjust mode closed.");
            }
            
            start_velocity_collect_mode = 0;
        }

    }

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr tcp_velocity_sub;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr velocity_collect_switch_sub;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);  // 初始化 ROS 2
    auto node = std::make_shared<VelocityCollectSubscriber>();  // 创建订阅者节点
    rclcpp::spin(node);  // 保持节点运行，监听消息
    rclcpp::shutdown();  // 关闭 ROS 2
    return 0;
}