import os
import yaml
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    tcpTolink6_yaml_path = LaunchConfiguration("tcpTolink6_yaml_path")
    tcpTolink6_yaml_path_cmd = DeclareLaunchArgument("tcpTolink6_yaml_path", default_value=
                                                  os.path.join(get_package_share_directory("ur3_moveit"),"config","tcpTolink6.yaml"))
    
    planning_filename = LaunchConfiguration("planning_filename")
    planning_filename_cmd = DeclareLaunchArgument("planning_filename", default_value="path.txt")

    including_pose_estimation = LaunchConfiguration("including_pose_estimation")
    including_pose_estimation_cmd = DeclareLaunchArgument("including_pose_estimation", default_value="false")

    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur3", "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur3", "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur3", "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur3", "visual_parameters.yaml"]
    )
    
    # moveit_config = MoveItConfigsBuilder("abb_ros2").to_moveit_configs()
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"]),
            " ",
            "robot_ip:=xxx.yyy.zzz.www",
            " ",
            "joint_limit_params:=",
            joint_limit_params,
            " ",
            "kinematics_params:=",
            kinematics_params,
            " ",
            "physical_params:=",
            physical_params,
            " ",
            "visual_params:=",
            visual_params,
            " ",
            "safety_limits:=",
            "true",
            " ",
            "safety_pos_margin:=",
            "0.15",
            " ",
            "safety_k_position:=",
            "20",
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            "ur3",
            " ",
            "script_filename:=ros_control.urscript",
            " ",
            "input_recipe_filename:=rtde_input_recipe.txt",
            " ",
            "output_recipe_filename:=rtde_output_recipe.txt",
            " ",
            "prefix:=",
            '""',
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    
    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare("ur_moveit_config"), "config", "kinematics.yaml"]
    )

    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ur_moveit_config"), "srdf", "ur.srdf.xacro"]
            ),
            " ",
            "name:=",
            # Also ur_type parameter could be used but then the planning group names in yaml
            # configs has to be updated!
            "ur",
            " ",
            "prefix:=",
            '""',
            " ",
        ]
    )
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

    # MoveGroupInterface demo executable
    move_group_demo = Node(
        name="ur3_moveit",
        package="ur3_moveit",
        executable="ur3_moveit",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            tcpTolink6_yaml_path,
            {"planning_filename": planning_filename,
             "including_pose_estimation": including_pose_estimation}
        ],
    )

    return LaunchDescription([
        tcpTolink6_yaml_path_cmd,
        planning_filename_cmd,
        including_pose_estimation_cmd,
        move_group_demo,
        ])