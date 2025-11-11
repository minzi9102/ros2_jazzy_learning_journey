from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    Shutdown,  # ⭐ 添加这行
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import (
    AndSubstitution,
    LaunchConfiguration,
    NotSubstitution,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable


def launch_setup(context):
    example_file = LaunchConfiguration("example_file")
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")

    static_tf = Node(
        package="tf2_ros",#<- 需要添加依赖包tf2_ros
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "base", "--child-frame-id", "base_link_inertia"],
    )

    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
        #创建 MoveItConfigsBuilder 类的一个实例（对象）。你可以把它想象成“启动一个 MoveIt 配置的装配工”。
        #它被告知去 ur_moveit_config 包里寻找名为 ur 的机器人的配置文件。
        #构建器将在这个包的 config子目录中查找所有必需的 .yaml文件。
        .robot_description_semantic(Path("srdf") / "ur.srdf.xacro", {"name": ur_type})
    
        .moveit_cpp(
            file_path=get_package_share_directory("motion_planning_py")#<- 可以修改为FindPackageShare
            # + "/config/motion_planning_python_api_tutorial.yaml"
            + "/config/motion_planning_python_api_tutorial.yaml"
        )
        .to_moveit_configs()
        )

    moveit_py_node = Node(
        name="moveit_py",
        package="motion_planning_py",
        executable=example_file,
        output="both",
        parameters=[moveit_config.to_dict()],
        # on_exit=Shutdown(),  # ⭐ 关键：节点退出时关闭整个 launch
    )

    rviz_node = Node(
    package="rviz2",
    executable="rviz2",
    name="rviz2",
    output="log",
    arguments=["-d", get_package_share_directory("motion_planning_py") + "/config/visual.rviz"],
    parameters=[
        moveit_config.to_dict(),
        {"use_sim_time": True}  # 如果使用 mock hardware
    ]
    )

    nodes_to_start = [
        rviz_node,
        static_tf,
        moveit_py_node,
        
    ]

    return nodes_to_start

def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",#定义机器人的具体型号（如 "ur3", "ur5e" 等）
            description="Type/series of used UR robot.",
            default_value="ur3",
            choices=[
                "ur3",
                "ur5",
                "ur10",
                "ur3e",
                "ur5e",
                "ur7e",
                "ur10e",
                "ur12e",
                "ur16e",
                "ur8long",
                "ur15",
                "ur18",
                "ur20",
                "ur30",
            ],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip", 
            default_value="0.0.0.0",
            description="IP address by which the robot can be reached."# 指定真实机器人的 IP 地址。
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
        "example_file",
        default_value="motion_planning_python_api_planning_scene.py",
        description="Python API tutorial file name",
    )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
