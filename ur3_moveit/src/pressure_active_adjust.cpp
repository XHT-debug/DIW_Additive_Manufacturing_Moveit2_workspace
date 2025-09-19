#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <vector>
#include <string>

#include <std_msgs/msg/int32.hpp>

// 定义一个结构体，用于存储时间戳和气压值
struct PressureData {
    double timestamp;  // 时间戳
    int pressure;   // 气压值
};

// 全局变量
std::ifstream file;  // 文件流
std::string filepath = "/home/xht/moveit2_workspace/src/ur3_moveit/config/pressure_adjust/pressure_data.txt";  // 文件名
rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pressure_active_adjust_pub;  // 气压值发布者
std::vector<PressureData> pressure_data_list;  // 存储文件中的数据
size_t current_index = 0;  // 当前处理的数据索引
bool start_pressure_active_adjust_mode = false;  // 控制模式是否开启
std_msgs::msg::Int32 msg_pressure;

double start_time;


// 读取文件并解析数据
void readPressureDataFromFile(const std::string& filename) {
    file.open(filename);
    if (!file.is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("pressure_active_adjust"), "Failed to open file: %s", filename.c_str());
        return;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream ss(line);
        std::string token;
        std::vector<std::string> tokens;

        while (std::getline(ss, token, ' ')) {
            tokens.push_back(token);
        }

        if (tokens.size() != 2) {
            RCLCPP_WARN(rclcpp::get_logger("pressure_active_adjust"), "Invalid line in file: %s", line.c_str());
            continue;
        }

        PressureData data;
        data.timestamp = std::stod(tokens[0]);  // 时间戳
        data.pressure = std::stod(tokens[1]);   // 气压值
        pressure_data_list.push_back(data);
    }

    file.close();
    RCLCPP_INFO(rclcpp::get_logger("pressure_active_adjust"), "Loaded %zu pressure data points from file.", pressure_data_list.size());
}

// 定时器回调函数，根据时间戳发布气压值
void publishPressureCallback_outside() {
    
    if (start_pressure_active_adjust_mode == false || current_index >= pressure_data_list.size()) {
        return;  // 如果模式未开启或数据已处理完，直接返回
    }

    double current_time = rclcpp::Clock().now().seconds();  // 获取当前时间（秒）
    
    double next_timestamp = pressure_data_list[current_index].timestamp;

    // 如果当前时间达到或超过文件中的时间戳，则发送气压值
    if (current_time - start_time >= next_timestamp - 0.2) {
        msg_pressure.data = pressure_data_list[current_index].pressure - 7;
        pressure_active_adjust_pub->publish(msg_pressure);
        
        RCLCPP_INFO(rclcpp::get_logger("pressure_active_adjust"), "Published pressure: %d at timestamp: %.2f", msg_pressure.data, next_timestamp);

        // 移动到下一条数据
        current_index += 5;
    }
}




class PressureActiveAdjustSubscriber : public rclcpp::Node
{
public:
    PressureActiveAdjustSubscriber()
        : Node("pressure_active_adjust")  // 设置节点名称
    {
        // 创建订阅者，订阅 "/pressure_active_adjust_control" 话题
        pressure_adjust_start_sub = this->create_subscription<std_msgs::msg::Int32>(
            "/pressure_active_adjust_control", 10,  // 话题名称和队列大小
            std::bind(&PressureActiveAdjustSubscriber::pressure_active_adjust_control_callback, this, std::placeholders::_1));

        pressure_active_adjust_pub = this->create_publisher<std_msgs::msg::Int32>("/pressure_adjust", 10);

        // 创建定时器，每秒调用一次回调函数
        timer = this->create_wall_timer(std::chrono::milliseconds(8), std::bind(&PressureActiveAdjustSubscriber::publishPressureCallback, this));

        // 读取文件中的数据
        readPressureDataFromFile(filepath);

    }

private:
     // 回调函数：处理 "/pressure_active_adjust_control" 话题的消息
     void pressure_active_adjust_control_callback(const std_msgs::msg::Int32::SharedPtr msg) {

        if (msg->data == 1)
        {
            current_index = 0;  // 重置索引，重新开始处理文件数据
            RCLCPP_INFO(rclcpp::get_logger("pressure_active_adjust"), "Pressure active adjust mode started.");
            start_time = rclcpp::Clock().now().seconds();
            start_pressure_active_adjust_mode = true;
        }
        else if (msg->data == 0)
        {
            start_pressure_active_adjust_mode = false;
            RCLCPP_INFO(rclcpp::get_logger("pressure_active_adjust"), "Pressure active adjust mode stopped.");
        }

    }

    // void pressureActiveAdjustControlCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    //     pressureActiveAdjustControlCallback(msg);
    // }

    void publishPressureCallback() {
        publishPressureCallback_outside();
    }

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr pressure_adjust_start_sub;
    rclcpp::TimerBase::SharedPtr timer;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);  // 初始化 ROS 2
    auto node = std::make_shared<PressureActiveAdjustSubscriber>();  // 创建订阅者节点
    rclcpp::spin(node);  // 保持节点运行，监听消息
    rclcpp::shutdown();  // 关闭 ROS 2
    return 0;
}