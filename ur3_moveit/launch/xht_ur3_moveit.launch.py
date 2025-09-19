import os

from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    
    start_node_arg = DeclareLaunchArgument(
        'enable_node1',
        default_value='True',
        description='Whether to start the node'
    )
    
    # node1实现订阅/joint_state的话题信息，利用雅可比矩阵转换得到末端TCP速度矢量信息
    node1 = Node(
        package = 'ur3_moveit',
        executable = 'tcp_velocity',      
        condition = IfCondition(LaunchConfiguration('start_node')) 
    )
    # node2实现：1.机械臂末端里程计发布；2.接受TCP速度矢量信息，计算标量速度信息；3.实时发布气压调控信息（延迟0.2-0.4s）
    node2 = Node(
        package = "ur3_moveit",
        executable = "velocity_output"
    )
    
    node3 = Node(
        package = "ur3_moveit",
        executable = "ur3_moveit"
    )

    # 创建LaunchDescription对象launch_description,用于描述launch文件
    launch_description = LaunchDescription(
        [start_node_arg, node1, node2, node3])
    
    # 返回让ROS2根据launch描述执行节点
    return launch_description
