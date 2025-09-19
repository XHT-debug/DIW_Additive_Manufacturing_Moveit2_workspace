#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <chrono>
#include <memory>

class TrajectoryVelocityController : public rclcpp::Node
{
public:
    TrajectoryVelocityController() : Node("trajectory_velocity_controller")
    {
        // 声明参数
        this->declare_parameter<std::string>("trajectory_file", 
            "/home/xht/moveit2_workspace/src/ur3_moveit/config/trajectory_output/trajectory_postprocess_20250915_212010.txt");
        this->declare_parameter<double>("playback_speed", 1.0);
        this->declare_parameter<bool>("loop_trajectory", false);
        
        // 获取参数
        trajectory_file_ = this->get_parameter("trajectory_file").as_string();
        playback_speed_ = this->get_parameter("playback_speed").as_double();
        loop_trajectory_ = this->get_parameter("loop_trajectory").as_bool();
        
        // 创建发布者
        velocity_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/forward_velocity_controller/commands", 10);
        
        // 加载轨迹数据
        if (!loadTrajectoryData()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load trajectory data from: %s", trajectory_file_.c_str());
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Loaded %zu trajectory points from: %s", 
                   trajectory_data_.size(), trajectory_file_.c_str());
        
        // 创建定时器，以125Hz频率发布（8ms间隔）
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(8),
            std::bind(&TrajectoryVelocityController::publishVelocityCommand, this));
        
        // 初始化时间
        start_time_ = this->now();
        current_point_index_ = 0;
        
        RCLCPP_INFO(this->get_logger(), "Trajectory velocity controller started");
    }

private:
    struct TrajectoryPoint
    {
        double time;
        std::vector<double> velocities;  // 6个关节的速度
    };
    
    std::vector<TrajectoryPoint> trajectory_data_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::string trajectory_file_;
    double playback_speed_;
    bool loop_trajectory_;
    
    rclcpp::Time start_time_;
    size_t current_point_index_;
    
    bool loadTrajectoryData()
    {
        std::ifstream file(trajectory_file_);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open file: %s", trajectory_file_.c_str());
            return false;
        }
        
        std::string line;
        bool header_found = false;
        
        while (std::getline(file, line)) {
            // 跳过空行
            if (line.empty()) {
                continue;
            }
            
            // 查找数据行（包含时间戳的行，包括注释行）
            if (!header_found && line.find("时间,") != std::string::npos) {
                header_found = true;
                continue;
            }
            
            // 跳过其他注释行
            if (line[0] == '#') {
                continue;
            }
            
            if (header_found) {
                TrajectoryPoint point;
                std::istringstream iss(line);
                std::string token;
                std::vector<std::string> tokens;
                
                // 分割逗号分隔的数据
                while (std::getline(iss, token, ',')) {
                    tokens.push_back(token);
                }
                
                // 检查是否有足够的数据列（至少需要时间 + 6个位置 + 6个速度）
                if (tokens.size() >= 13) {
                    try {
                        point.time = std::stod(tokens[0]);
                        
                        // 提取6个关节的速度数据（第7-12列）
                        point.velocities.resize(6);
                        for (int i = 0; i < 6; i++) {
                            point.velocities[i] = std::stod(tokens[7 + i]);
                        }
                        
                        trajectory_data_.push_back(point);
                    } catch (const std::exception& e) {
                        RCLCPP_WARN(this->get_logger(), "Failed to parse line: %s", line.c_str());
                        continue;
                    }
                }
            }
        }
        
        file.close();
        
        if (trajectory_data_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No valid trajectory data found in file");
            return false;
        }
        
        return true;
    }
    
    void publishVelocityCommand()
    {
        if (trajectory_data_.empty()) {
            return;
        }
        
        // 计算当前时间（考虑播放速度）
        auto current_time = this->now();
        double elapsed_time = (current_time - start_time_).seconds() * playback_speed_;
        
        // 查找当前时间对应的轨迹点
        size_t target_index = findTrajectoryPoint(elapsed_time);
        
        if (target_index < trajectory_data_.size()) {
            // 发布速度指令
            auto velocity_msg = std_msgs::msg::Float64MultiArray();
            velocity_msg.data = trajectory_data_[target_index].velocities;
            
            velocity_publisher_->publish(velocity_msg);
            
            // 更新当前点索引
            current_point_index_ = target_index;
            
            // 检查是否到达轨迹末尾
            if (target_index == trajectory_data_.size() - 1) {
                if (loop_trajectory_) {
                    // 重新开始播放
                    start_time_ = this->now();
                    current_point_index_ = 0;
                    RCLCPP_INFO(this->get_logger(), "Trajectory completed, restarting...");
                } else {
                    // 停止定时器
                    timer_->cancel();
                    RCLCPP_INFO(this->get_logger(), "Trajectory playback completed");
                }
            }
        }
    }
    
    size_t findTrajectoryPoint(double target_time)
    {
        // 从当前索引开始搜索，提高效率
        for (size_t i = current_point_index_; i < trajectory_data_.size(); i++) {
            if (trajectory_data_[i].time >= target_time) {
                return i;
            }
        }
        
        // 如果没找到，返回最后一个点
        return trajectory_data_.size() - 1;
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<TrajectoryVelocityController>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
