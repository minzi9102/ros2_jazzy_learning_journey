import os
import yaml

from pathlib import Path

from launch import LaunchDescription# ->需要launch
from launch.actions import DeclareLaunchArgument, RegisterEventHandler# ->需要launch
from launch.conditions import IfCondition# ->需要launch
from launch.event_handlers import OnProcessExit# ->需要launch
from launch.substitutions import (# ->需要launch
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node# ->需要launch_ros
from launch_ros.substitutions import FindPackageShare# ->需要launch_ros

from moveit_configs_utils import MoveItConfigsBuilder# ->需要moveit_configs_utils

from ament_index_python.packages import get_package_share_directory# ->需要ament_index_python

# ---------------------------------------------------------------------------
# 辅助函数：声明所有启动参数
# ---------------------------------------------------------------------------
def declare_arguments():
    """
    声明此启动文件接受的所有参数。
    将声明与逻辑分离，使代码更清晰。
    """
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="ur_type",
                default_value="ur3",
                description="Universal Robots 机械臂型号（ur3、ur5 或 ur10）。",
                choices=["ur3", "ur5", "ur10"],
            ),
        ]
    )

# ---------------------------------------------------------------------------
# 主入口函数：generate_launch_description
# ---------------------------------------------------------------------------
def generate_launch_description():

    """
    ROS 2 启动系统的主要入口点。
    此函数在“设置阶段”被调用，以创建“蓝图”。
    """
    # -------------------------------------------------
    # 步骤 1: 创建“占位符”(LaunchConfiguration)
    # -------------------------------------------------
    # 为每个已声明的参数创建占位符。
    # 这些对象本身没有值，它们只是“引用”启动上下文中的值。
    ur_type = LaunchConfiguration("ur_type")

    # -------------------------------------------------
    # 步骤 2: 准备“配方”(Substitutions / Recipes)
    # -------------------------------------------------
    # 这是一个动态“配方”，用于在“执行阶段”运行 xacro
    # 以便根据 'robot_model' 占位符生成 URDF。
    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")# ->需要ur_moveit_config
        .robot_description_semantic(Path("srdf") / "ur.srdf.xacro", {"name": ur_type})
        .to_moveit_configs()
    )

    # -------------------------------------------------
    # 步骤 3: 创建主启动描述 (LD)
    # -------------------------------------------------
    ld = LaunchDescription()

    # -------------------------------------------------
    # 步骤 4: 声明与验证参数
    # -------------------------------------------------
    # 调用辅助函数，将所有“参数声明”添加到 LD 中。
    # ROS 2 系统会在此刻验证命令行传入的参数。
    ld.add_entity(declare_arguments())



    # -------------------------------------------------
    # 步骤 5: 定义节点“蓝图”
    # -------------------------------------------------
    # 节点1: ur_model_and_state 节点
    ur_model_and_state = Node(
        package="ur_model_and_state",
        executable="ur_model_and_state",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )
    # -------------------------------------------------
    # 步骤 6: 定义“事件处理器”(启动顺序)
    # -------------------------------------------------
    ld.add_action(ur_model_and_state)
    # -------------------------------------------------
    # 步骤 7: 返回“蓝图”
    # -------------------------------------------------
    return ld