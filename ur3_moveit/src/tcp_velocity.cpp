#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/SVD>

#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr tcp_velocity_pub;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr tcp_position_pub;
// 定义一个函数来计算雅可比矩阵
std::vector<std::vector<double>> fastJacob(const std::vector<double>& q) {
    // UR3 机械臂参数
    const double a2 = -243.65;
    const double a3 = -213.0;
    const double d1 = 151.9;
    const double d2 = 119.85;
    const double d4 = -9.45;
    const double d5 = 83.4;
    const double d6 = 82.4;

    // 初始化雅可比矩阵
    std::vector<std::vector<double>> J(6, std::vector<double>(6, 0.0));

    // 计算雅可比矩阵的每个元素
    J[0][0] = d6 * (cos(q[0]) * cos(q[4]) + cos(q[1] + q[2] + q[3]) * sin(q[0]) * sin(q[4])) + d2 * cos(q[0]) + d4 * cos(q[0]) - a3 * cos(q[1] + q[2]) * sin(q[0]) - a2 * cos(q[1]) * sin(q[0]) - d5 * sin(q[1] + q[2] + q[3]) * sin(q[0]);
    J[1][0] = d6 * (cos(q[4]) * sin(q[0]) - cos(q[1] + q[2] + q[3]) * cos(q[0]) * sin(q[4])) + d2 * sin(q[0]) + d4 * sin(q[0]) + a3 * cos(q[1] + q[2]) * cos(q[0]) + a2 * cos(q[0]) * cos(q[1]) + d5 * sin(q[1] + q[2] + q[3]) * cos(q[0]);
    J[2][0] = 0.0;
    J[3][0] = 0.0;
    J[4][0] = 0.0;
    J[5][0] = 1.0;

    J[0][1] = -cos(q[0]) * (a3 * sin(q[1] + q[2]) + a2 * sin(q[1]) - d5 * cos(q[1] + q[2] + q[3]) - d6 * sin(q[1] + q[2] + q[3]) * sin(q[4]));
    J[1][1] = -sin(q[0]) * (a3 * sin(q[1] + q[2]) + a2 * sin(q[1]) - d5 * cos(q[1] + q[2] + q[3]) - d6 * sin(q[1] + q[2] + q[3]) * sin(q[4]));
    J[2][1] = a3 * cos(q[1] + q[2]) + a2 * cos(q[1]) + d5 * (cos(q[1] + q[2]) * sin(q[3]) + sin(q[1] + q[2]) * cos(q[3])) - d6 * sin(q[4]) * (cos(q[1] + q[2]) * cos(q[3]) - sin(q[1] + q[2]) * sin(q[3]));
    J[3][1] = sin(q[0]);
    J[4][1] = -cos(q[0]);
    J[5][1] = 0.0;

    J[0][2] = cos(q[0]) * (d5 * cos(q[1] + q[2] + q[3]) - a3 * sin(q[1] + q[2]) + d6 * sin(q[1] + q[2] + q[3]) * sin(q[4]));
    J[1][2] = sin(q[0]) * (d5 * cos(q[1] + q[2] + q[3]) - a3 * sin(q[1] + q[2]) + d6 * sin(q[1] + q[2] + q[3]) * sin(q[4]));
    J[2][2] = a3 * cos(q[1] + q[2]) + d5 * sin(q[1] + q[2] + q[3]) - d6 * cos(q[1] + q[2] + q[3]) * sin(q[4]);
    J[3][2] = sin(q[0]);
    J[4][2] = -cos(q[0]);
    J[5][2] = 0.0;

    J[0][3] = cos(q[0]) * (d5 * cos(q[1] + q[2] + q[3]) + d6 * sin(q[1] + q[2] + q[3]) * sin(q[4]));
    J[1][3] = sin(q[0]) * (d5 * cos(q[1] + q[2] + q[3]) + d6 * sin(q[1] + q[2] + q[3]) * sin(q[4]));
    J[2][3] = d5 * sin(q[1] + q[2] + q[3]) - d6 * cos(q[1] + q[2] + q[3]) * sin(q[4]);
    J[3][3] = sin(q[0]);
    J[4][3] = -cos(q[0]);
    J[5][3] = 0.0;

    J[0][4] = -d6 * (sin(q[0]) * sin(q[4]) + cos(q[1] + q[2] + q[3]) * cos(q[0]) * cos(q[4]));
    J[1][4] = d6 * (cos(q[0]) * sin(q[4]) - cos(q[1] + q[2] + q[3]) * cos(q[4]) * sin(q[0]));
    J[2][4] = -d6 * sin(q[1] + q[2] + q[3]) * cos(q[4]);
    J[3][4] = sin(q[1] + q[2] + q[3]) * cos(q[0]);
    J[4][4] = sin(q[1] + q[2] + q[3]) * sin(q[0]);
    J[5][4] = -cos(q[1] + q[2] + q[3]);

    J[0][5] = 0.0;
    J[1][5] = 0.0;
    J[2][5] = 0.0;
    J[3][5] = cos(q[4]) * sin(q[0]) - cos(q[1] + q[2] + q[3]) * cos(q[0]) * sin(q[4]);
    J[4][5] = -cos(q[0]) * cos(q[4]) - cos(q[1] + q[2] + q[3]) * sin(q[0]) * sin(q[4]);
    J[5][5] = -sin(q[1] + q[2] + q[3]) * sin(q[4]);

    // 将小于阈值的元素设为0
    const double eps = 1e-5;
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            if (std::abs(J[i][j]) < eps) {
                J[i][j] = 0.0;
            }
        }
    }

    return J;
}

Eigen::MatrixXd joint_transform_matrix(const Eigen::MatrixXd& theta)
{
    Eigen::VectorXd alpha_dynamic(6);
    Eigen::VectorXd a_dynamic(6);
    Eigen::VectorXd d_dynamic(6);
    const double PI = 3.14159265358979323846; // 双精度

    alpha_dynamic << PI/2, 0, 0, PI/2, -PI/2, 0;
    a_dynamic << 0, -0.24376288177085403, -0.21344066092027106, 0, 0, 0;
    d_dynamic << 0.15187159190950295, 0, 0, 0.11209388124617316, 0.085378607490435937, 0.08241227224463675;
    Eigen::MatrixXd T_i_1_i_assemble = Eigen::MatrixXd::Zero(4, 4 * 7);
    for (int i = 1; i <= 6; i++) {
        double cos_theta = std::cos(std::fmod(theta(i-1, 0), 2 * PI));
        double sin_theta = std::sin(std::fmod(theta(i-1, 0), 2 * PI));
        double cos_alpha = std::cos(std::fmod(alpha_dynamic(i-1, 0), 2 * PI));
        double sin_alpha = std::sin(std::fmod(alpha_dynamic(i-1, 0), 2 * PI));
        T_i_1_i_assemble.block<4, 4>(0, 4 * (i - 1)) << cos_theta, -sin_theta * cos_alpha, sin_theta * sin_alpha, a_dynamic(i-1) * cos_theta,
            sin_theta, cos_theta * cos_alpha, -cos_theta * sin_alpha, a_dynamic(i-1) * sin_theta,
            0, sin_alpha, cos_alpha, d_dynamic(i-1),
            0, 0, 0, 1;
    }

    T_i_1_i_assemble.block<4, 4>(0, 4 * 6) << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    return T_i_1_i_assemble;
}

class TcpVelocitySubscriber : public rclcpp::Node
{
public:
    TcpVelocitySubscriber()
        : Node("tcp_velocity")  // 设置节点名称
    {
        // 创建订阅者，订阅 "/joint_states" 话题
        joint_states_sub = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,  // 话题名称和队列大小
            std::bind(&TcpVelocitySubscriber::joint_state_callback, this, std::placeholders::_1));

        tcp_velocity_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/tcp_velocity", 10);
        tcp_position_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/tcp_position_by_collision", 10);

    }

private:
    // 回调函数：处理 "/joint_state" 话题的消息
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        // 确保消息中包含足够的关节数据
        
        if (msg->position.size() < 6 || msg->velocity.size() < 6) {
            RCLCPP_ERROR(this->get_logger(), "Joint state message does not contain enough data.");
            return;
        }

        // 提取关节位置数据
        std::vector<double> joint_positions = {msg->position[5], msg->position[0], msg->position[1], msg->position[2], msg->position[3], msg->position[4]};
    
        std::vector<double> joint_velocitys = {msg->velocity[5], msg->velocity[0], msg->velocity[1], msg->velocity[2], msg->velocity[3], msg->velocity[4]};

        std::vector<double> TCP_velocity(8, 0.0);  // 初始化为大小为6的向量，所有元素为0
    
        // 计算雅可比矩阵
        std::vector<std::vector<double>> J(6, std::vector<double>(6, 0.0));
        
        J = fastJacob(joint_positions);
        
        for(int i = 0; i < 5; i++){
    
            for(int j = 0; j < 6; j++){
    
                TCP_velocity[i] += J[i][j]*joint_velocitys[j];
    
            }
    
        }
    
        auto msg_tcp_velocity = std_msgs::msg::Float64MultiArray();
    
        // 雅各比矩阵对应rad转换mm/s
        msg_tcp_velocity.data = {TCP_velocity[0] * 0.001, TCP_velocity[1] * 0.001, TCP_velocity[2] * 0.001, TCP_velocity[3] * 0.001, TCP_velocity[4] * 0.001, TCP_velocity[5]*0.001}; 
        
        // /tcp_velocity的7，8位数据储存/joint_states的时间header
        msg_tcp_velocity.data.push_back(msg->header.stamp.sec);  // 秒部分
        msg_tcp_velocity.data.push_back(msg->header.stamp.nanosec);  // 纳秒部分
        
        tcp_velocity_pub->publish(msg_tcp_velocity);

        Eigen::MatrixXd position = Eigen::MatrixXd::Zero(6, 1);
        position << msg->position[0], msg->position[1], msg->position[2], msg->position[3], msg->position[4], msg->position[5];
        Eigen::MatrixXd T_i_1_i_assemble = joint_transform_matrix(position);

        Eigen::MatrixXd T_6 = Eigen::MatrixXd::Zero(4, 1);
        T_6 << 0, 0, 0, 1;
        

        for(int i=1;i<=6;i++)
        {
            T_6 = T_i_1_i_assemble.block<4, 4>(0, 4*(6 - i)) * T_6;
        }

        geometry_msgs::msg::PoseStamped msg_tcp_position;
        msg_tcp_position.header.stamp = msg->header.stamp;
        msg_tcp_position.header.frame_id = "base_link";
        msg_tcp_position.pose.position.x = T_6(0, 0);
        msg_tcp_position.pose.position.y = T_6(1, 0);
        msg_tcp_position.pose.position.z = T_6(2, 0);
        msg_tcp_position.pose.orientation.x = 0;
        msg_tcp_position.pose.orientation.y = 0;
        msg_tcp_position.pose.orientation.z = 0;
        msg_tcp_position.pose.orientation.w = 1;

        tcp_position_pub->publish(msg_tcp_position);
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);  // 初始化 ROS 2
    auto node = std::make_shared<TcpVelocitySubscriber>();  // 创建订阅者节点
    rclcpp::spin(node);  // 保持节点运行，监听消息
    rclcpp::shutdown();  // 关闭 ROS 2
    return 0;
}