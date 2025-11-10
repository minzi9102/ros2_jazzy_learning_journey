from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
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
    # Initialize Arguments
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    example_file = LaunchConfiguration("example_file")
    # General arguments
    description_launchfile = LaunchConfiguration("description_launchfile")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    controller_spawner_timeout = LaunchConfiguration("controller_spawner_timeout")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")
    launch_rviz = LaunchConfiguration("launch_rviz")
    # rviz_config_file = LaunchConfiguration("rviz_config_file")
    headless_mode = LaunchConfiguration("headless_mode")
    launch_dashboard_client = LaunchConfiguration("launch_dashboard_client")


    # 🧠核心：控制器与驱动节点

    """
    加载 controllers_file (YAML文件) 中定义的所有控制器（如 joint_state_broadcaster 和 joint_trajectory_controller）。
    它从硬件（ur_robot_driver）读取状态，并将其喂给控制器；同时，它接收控制器的命令，并将其发送给硬件。
    依赖参数: controllers_file
    依赖参数: update_rate_config_file
    """
    controllers_file = PathJoinSubstitution([
                FindPackageShare("ur_robot_driver"), #<- 需要添加依赖包ur_robot_driver
                "config", 
                "ur_controllers.yaml"
            ])
    update_rate_config_file = [PathJoinSubstitution(
                    [
                        FindPackageShare("ur_robot_driver"),
                        "config",
                    ]
                ),
                "/",
                ur_type,
                "_update_rate.yaml",]
            

    control_node = Node(
        package="controller_manager",#<- 需要添加依赖包controller_manager,该节点启动后默认名字为controller_manager
        executable="ros2_control_node",
        parameters=[
            update_rate_config_file,
            ParameterFile(controllers_file, allow_substs=True),
            # We use the tf_prefix as substitution in there, so that's why we keep it as an
            # argument for this launchfile
        ],
        output="screen",
    )

    """
    功能: 连接真实机器人的“钥匙”。
    详细: 启动时，它会一次性通过网络将一个名为 external_control.urscript 的脚本程序发送到 UR 机器人控制器上。
    这个脚本必须在机器人上运行，ros2_control 才能通过实时端口 (50001-50003) 与机器人通信。
    """
    urscript_interface = Node(
        package="ur_robot_driver",#<- 需要添加依赖包ur_robot_driver
        executable="urscript_interface",
        parameters=[{"robot_ip": robot_ip}],
        output="screen",
        condition=UnlessCondition(use_mock_hardware),#<- use_mock_hardware为true时，不启动
    )

    """
    controller_spawners (控制器启动器)

    功能: 告诉“大脑”开始工作的“启动命令”。

    详细: control_node 启动时只是一个空壳。
    spawner 是一个运行后即退出的程序，它连接到 control_node 并告诉它：
    “嘿，请从你的配置中加载并激活这个控制器列表（例如 joint_state_broadcaster 和 scaled_joint_trajectory_controller）”。

    这里启动了两个 spawner：

        一个用于激活 controllers_active 列表中的控制器（主要是状态广播器和默认的运动控制器）。

        另一个用于加载 controllers_inactive 列表中的控制器（但不激活），使它们可以在以后被动态切换。
    """
    # Spawn controllers
    def controller_spawner(controllers, active=True):
        inactive_flags = ["--inactive"] if not active else []
        return Node(
            package="controller_manager",#<- 需要添加依赖包controller_manager
            executable="spawner",
            arguments=[
                "--controller-manager",
                "/controller_manager",
                "--controller-manager-timeout",
                controller_spawner_timeout,
            ]
            + inactive_flags
            + controllers,
        )

    controllers_active = [
        "joint_state_broadcaster",
        # 从硬件读取所有关节的当前角度，并将其发布到 /joint_states 话题。
        # 这是 robot_state_publisher (RSP) 节点工作的前提，RSP 节点（在 rsp 启动文件中）会订阅此话题，
        # 使你的 URDF 模型在 RViz 中动起来。
        "io_and_status_controller",
        # 从硬件读取机器人的I/O状态（例如数字/模拟输入和输出引脚）和机器人状态（例如是否处于保护性停止状态），并将其发布为 ROS 2 话题。
        "speed_scaling_state_broadcaster",
        # 读取并发布示教器上的速度缩放滑块的当前值（例如 50% 或 100%）。这对于 scaled_..._controller（见下文）至关重要。
        "force_torque_sensor_broadcaster",
        # 读取机器人法兰（手腕）上的力/扭矩传感器数据，并将其发布为 ROS 2 话题。
        "tcp_pose_broadcaster",
        # 读取机器人控制器计算的工具中心点 (Tool Center Point, TCP) 的当前姿态，并将其作为 TF 变换发布。
        "ur_configuration_controller",
        # 用于在运行时读取和设置机器人的某些特定配置（如工具的负载、TCP 设置等）的服务。
    ]
    controllers_inactive = [
        # 这个列表中的控制器是互斥的 (Mutually Exclusive)。
        # 你不能同时激活两个运动控制器（例如，你不能同时命令机器人“跟踪一个轨迹”又“保持一个速度”）。
        # 这个启动文件（ur_control.launch.py）将它们全部加载（load），但默认保持非激活（inactive）。

        "scaled_joint_trajectory_controller",
        # 接受一个关节轨迹（FollowJointTrajectory Action，通常来自 MoveIt 2），并使机器人执行它。
        # 它会“尊重” speed_scaling_state_broadcaster 报告的速度。如果你在示教器上把速度调到 10%，这个控制器会自动减慢运动，这在调试时非常安全。
        "joint_trajectory_controller",
        # 功能与上面相同，但它会忽略示教器上的速度缩放，始终以程序设定的 100% 速度运行。
        "forward_velocity_controller",# “正向控制器”，允许你绕过轨迹，直接向 ros2_control 发送实时的关节速度指令。
        "forward_position_controller",# “正向控制器”，允许你绕过轨迹，直接向 ros2_control 发送实时的关节位置指令。
        # "forward_effort_controller",# “正向控制器”，允许你绕过轨迹，直接向 ros2_control 发送实时的关节力矩指令。
        #ur3机械臂无法直接控制实时关节力矩。其硬件架构和控制系统设计决定了它只能通过位置/速度接口进行间接力控，而非直接力矩控制。
        "force_mode_controller",
        # 激活 UR 机器人内置的“力模式”，允许你命令机器人沿某个轴施加一个特定的力（例如 10 牛顿），而不是移动到一个特定的位置。这在装配、打磨等任务中很有用。
        "passthrough_trajectory_controller",
        # 一个特殊的轨迹控制器，它不对轨迹进行过多处理，直接将其传递给机器人。
        "freedrive_mode_controller",
        # 通过 ROS 2 服务来激活或关闭机器人的**“自由驱动”**模式（即你按下机器人法兰上的按钮，用手拖动机器人的模式）。
        "tool_contact_controller",
        # 一个用于检测工具接触的控制器。
    ]
    if activate_joint_controller.perform(context) == "true":
        # 从这个列表中拿出一个控制器（默认为 scaled_joint_trajectory_controller），并将其移动到 active 列表中，以确保在启动时有一个（且只有一个）运动控制器处于激活状态。
        controllers_active.append(initial_joint_controller.perform(context))
        controllers_inactive.remove(initial_joint_controller.perform(context))

    if use_mock_hardware.perform(context) == "true":
        controllers_active.remove("tcp_pose_broadcaster")
        # tcp_pose_broadcaster唯一工作是去读取UR机器人控制柜（即PolyScope示教器系统）内部自己计算的TCP姿态值，然后将其作为TF变换发布出来。
        # 模拟硬件 (mock hardware) 无法提供真实机器人控制器才能计算出的数据,因此在使用模拟硬件时，必须移除该控制器。
        # 在模拟模式下，唯一的TCP姿态来源是 robot_state_publisher (RSP) 节点，
        # 它通过 joint_state_broadcaster 发布的关节角度和URDF模型在ROS 2端自己计算TCP位置。
    controller_spawners = [
        controller_spawner(controllers_active),
        controller_spawner(controllers_inactive, active=False),
    ]


    """
    rsp (Robot State Publisher - 机器人状态发布器)

    功能: “模型动画师”。

    详细: 这是一个 IncludeLaunchDescription，它会启动 robot_state_publisher 节点。
    此节点订阅 /joint_states 话题（由 joint_state_broadcaster 发布），
    并使用机器人的 URDF 模型，计算并广播所有连杆（link）之间的 TF 坐标变换。
    """
    rsp = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(description_launchfile),
        launch_arguments={
            "robot_ip": robot_ip,
            "ur_type": ur_type,
        }.items(),
    )
    #🛠️ 辅助：工具与状态
    """
    dashboard_client_node (仪表盘客户端)

    功能: 机器人的“高级遥控器”。

    详细: 它连接到 UR 机器人的仪表盘服务（端口 29999），允许你通过 ROS Service 发送高级命令，
    如**“上电”、“释放刹车”、“启动程序”或“关闭电源”**。
    """
    dashboard_client_node = Node(
        package="ur_robot_driver",#<- 需要添加依赖包ur_robot_driver
        condition=IfCondition(
            AndSubstitution(launch_dashboard_client, NotSubstitution(use_mock_hardware))
        ),
        executable="dashboard_client",
        name="dashboard_client",
        output="screen",
        emulate_tty=True,
        parameters=[{"robot_ip": robot_ip}],
    )
    """
    robot_state_helper_node (机器人状态助手)

    功能: “自动化助手”。

    详细: 尤其在 headless_mode (无头模式) 下非常有用。它会监控机器人状态，
    并在需要时自动使用 dashboard_client_node 的服务来为机器人上电和释放刹车，
    使其准备好接收 ros2_control 的命令，无需人工操作示教器。
    """
    robot_state_helper_node = Node(
        package="ur_robot_driver",#<- 需要添加依赖包ur_robot_driver
        executable="robot_state_helper",
        name="ur_robot_state_helper",
        output="screen",
        condition=UnlessCondition(use_mock_hardware),
        parameters=[
            {"headless_mode": headless_mode},
            {"robot_ip": robot_ip},
        ],
    )
    """
    controller_stopper_node (控制器停止器)

    功能: “安全员”。

    详细: 这是一个极其重要的安全节点。它持续监控机器人的状态。
    如果机器人进入**“保护性停止”（例如碰撞或触发了安全限制），
    此节点会立即停止** control_node 中所有激活的运动控制器。
    这可以防止控制器在机器人停止时累积误差，避免在恢复时发生危险的突然运动。
    """
    controller_stopper_node = Node(
        package="ur_robot_driver",#<- 需要添加依赖包ur_robot_driver
        executable="controller_stopper_node",
        name="controller_stopper",
        output="screen",
        emulate_tty=True,
        condition=UnlessCondition(use_mock_hardware),
        parameters=[
            {"headless_mode": headless_mode},
            {"joint_controller_active": activate_joint_controller},
            {
                "consistent_controllers": [
                    "io_and_status_controller",
                    "force_torque_sensor_broadcaster",
                    "joint_state_broadcaster",
                    "speed_scaling_state_broadcaster",
                    "tcp_pose_broadcaster",
                    "ur_configuration_controller",
                ]
            },
        ],
    )

    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
        #创建 MoveItConfigsBuilder 类的一个实例（对象）。你可以把它想象成“启动一个 MoveIt 配置的装配工”。
        #它被告知去 ur_moveit_config 包里寻找名为 ur 的机器人的配置文件。
        #构建器将在这个包的 config子目录中查找所有必需的 .yaml文件。
        .robot_description_semantic(Path("srdf") / "ur.srdf.xacro", {"name": ur_type})
        .moveit_cpp(
            file_path=get_package_share_directory("motion_planning_py")#<- 可以修改为FindPackageShare
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
    )

    """
    rviz_node (RViz 可视化)

    功能: “3D 监视器”。

    详细: 启动 RViz 3D 可视化工具。它会订阅 rsp 发布的 TF 和其他话题，让你能够实时看到机器人的 3D 模型是如何运动的。
    """

    rviz_config_file = PathJoinSubstitution(
        # [FindPackageShare("motion_planning_py"), "config", "visual.rviz"]
        [FindPackageShare("ur_description"), "rviz", "view_robot.rviz"]
    )

    rviz_node = Node(
        package="rviz2",#<- 需要添加依赖包rviz2
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    static_tf = Node(
        package="tf2_ros",#<- 需要添加依赖包tf2_ros
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link_inertia"],
    )

    # """
    # trajectory_until_node (轨迹执行节点)

    # 功能: “特殊任务执行器”。

    # 详细: 这是一个辅助节点，它可能提供一个服务，用于“执行一个轨迹，直到某个（例如 I/O）条件被满足”，这是一种高级的运动控制。
    # """
    # trajectory_until_node = Node(
    #     package="ur_robot_driver",#<- 需要添加依赖包ur_robot_driver
    #     executable="trajectory_until_node",
    #     name="trajectory_until_node",
    #     output="screen",
    #     parameters=[
    #         {
    #             "motion_controller": initial_joint_controller,
    #         },
    #     ],
    # )

    nodes_to_start = [
        control_node,
        dashboard_client_node,
        robot_state_helper_node,
        controller_stopper_node,
        urscript_interface,
        rsp,
        rviz_node,
        static_tf,
        # moveit_py_node,
        # trajectory_until_node,
    ] + controller_spawners

    return nodes_to_start

    """
    🚀 它们如何协同工作 (启动流程)

    启动：control_node (大脑)、rsp (模型动画师) 和 rviz_node (监视器) 在 ROS 端启动。

    握手：urscript_interface (钥匙) 将控制脚本发送给机器人。dashboard_client_node (遥控器) 连接到机器人。

    上电：robot_state_helper_node (助手) 发现机器人处于空闲状态，于是它使用 dashboard_client_node 的服务来上电并释放刹车。

    激活：controller_spawners (启动命令) 依次运行。

    数据流：

        第一个 spawner 告诉 control_node：“激活 joint_state_broadcaster！”。

        control_node (通过 urscript_interface 建立的连接) 开始从机器人读取实时关节角度。

        joint_state_broadcaster 获取这些角度，并将其发布到 /joint_states 话题。

        rsp 节点侦听 /joint_states，计算 TF 变换，并广播它们。

        rviz_node 侦听 TF，rviz 中的机器人模型**“活了过来”**并与真实机器人同步。

    就绪：

        第一个 spawner 接着告诉 control_node：“激活 scaled_joint_trajectory_controller (运动控制器)！”

        第二个 spawner 告诉 control_node：“加载所有其他运动控制器，但保持非激活状态。”

    运行：

        系统现在完全就绪，可以接收来自 MoveIt 2 或其他 ROS 节点的运动指令了。

        controller_stopper_node (安全员) 在后台持续监控，确保一切安全。
    """


def generate_launch_description():

    # 为ROS 2启动文件定义所有可配置的启动参数
    # 允许用户在运行 ros2 launch 命令时，从命令行传入参数来改变启动行为，而无需修改这个Python文件本身。
    declared_arguments = []
    # UR specific arguments
    # A. 机器人核心参数（通常是必需的）
    
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
        default_value="motion_planning_python_api_tutorial.py",
        description="Python API tutorial file name",
    )
    )
    
    # B.安全与模拟参数（带默认值的开关）safety_limits, safety_pos_margin, safety_k_position 
    # 配置安全控制器的参数，如是否启用、安全边界等以及是否使用虚拟硬件。
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )

    """
    
    声明了 safety_limits 等参数,下方description_launchfile 变量指向的 ur_rsp.launch.py 文件（子级）也声明了同名的 safety_limits、safety_pos_margin 和 safety_k_position 参数。
    自动传递：当你 IncludeLaunchDescription 时，
    ROS 2 启动系统会自动将在父级中已声明（或在命令行中已设置）的、且在子级中也存在的所有参数值自动传递给子级启动文件。

    """

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",# 一个布尔开关（"true" 或 "false"），用于决定是连接真实机器人（"false"）还是启动一个模拟的硬件接口（"true"）
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "mock_sensor_commands",# 在 use_mock_hardware 为 "true" 时，进一步启用模拟的传感器命令。
            default_value="false",
            description="Enable mock command interfaces for sensors used for simple simulations. "
            "Used only if 'use_mock_hardware' parameter is true.",
        )
    )
    
    # General arguments
    # C. 配置文件路径（使用动态默认值）
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_launchfile",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ur_robot_driver"), "launch", "ur_rsp.launch.py"]# 机器人状态发布（RSP）启动文件。
            ),
            description="Launchfile (absolute path) providing the description. "
            "The launchfile has to start a robot_state_publisher node that "
            "publishes the description topic.",
        )
    )
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "rviz_config_file",
    #         default_value=PathJoinSubstitution(
    #             [FindPackageShare("ur_description"), "rviz", "view_robot.rviz"]# 默认的 RVIZ 配置文件
    #         ),
    #         description="RViz config file (absolute path) to use when launching rviz.",
    #     )
    # )
    
    # D. 控制与显示参数（开关和选项）
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "headless_mode",# 无头模式开关（"true" 或 "false"），用于在没有图形界面的服务器上运行。
            default_value="false",
            description="Enable headless mode for robot control",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="scaled_joint_trajectory_controller",# 指定默认加载并激活的关节控制器。
            choices=[
                "scaled_joint_trajectory_controller",
                "joint_trajectory_controller",
                "forward_velocity_controller",
                "forward_position_controller",
                "freedrive_mode_controller",
                "passthrough_trajectory_controller",
            ],
            description="Initially loaded robot controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "activate_joint_controller",# 布尔开关，决定是否在启动时自动激活 initial_joint_controller。
            default_value="true",
            description="Activate loaded joint controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")# 布尔开关，决定是否启动 RVIZ 可视化工具。
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_spawner_timeout",
            default_value="10",
            description="Timeout used when spawning controllers.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="",
            description="tf_prefix of the joint names, useful for "
            "multi-robot setup. If changed, also joint names in the controllers' configuration "
            "have to be updated.",
            # 这是一个TF（坐标变换）前缀。它会添加到所有ROS 2发布的机器人TF帧（如 base_link, tool0）和关节名称（如 shoulder_pan_joint）的前面。
            # 这在多机器人系统中至关重要。如果你有两个机器人，你可以将一个的 tf_prefix 设置为 "robot1"，另一个设置为 "robot2"。
            # 这样，它们的坐标系就会变成 robot1/base_link 和 robot2/base_link，从而在系统中避免了命名冲突。
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_dashboard_client",
            default_value="true",
            description="Launch Dashboard Client?",
            # 这是一个布尔开关，用于决定是否启动仪表盘客户端节点（dashboard_client）。
            # dashboard_client 提供了ROS Service（服务），允许你通过ROS命令来控制机器人的仪表盘（例如：上电、释放刹车、启动程序、关闭电源）。
            # 如果你不需要通过ROS来执行这些操作，可以将其设置为 "false" 来节省资源。
        )
    )
    
    # E. 通信端口与IP配置参数
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "reverse_ip", # 这是你的电脑（运行ROS驱动）的IP地址。机器人控制器需要知道这个IP，才能将数据**“反向”**发送回给驱动程序。
            default_value="0.0.0.0",
            description="IP that will be used for the robot controller to communicate back to the driver.",
            # 默认的 "0.0.0.0" 是一个特殊地址，意思是“监听所有可用的网络接口”。在大多数简单网络中这都有效。
            # 但在复杂网络（如使用Docker或有多个网卡）中，你可能需要将其明确设置为你电脑的IP地址（例如 "192.168.1.10"），以确保机器人知道该往哪里发送数据。
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "reverse_port",
            default_value="50001",
            description="Port that will be opened to send cyclic instructions from the driver to the robot controller.",
            # 这是主要的实时数据端口。机器人会周期性地（如每秒500次）将自己的状态（如关节角度、速度、I/O状态）通过这个端口发送回ROS驱动。
            # 这是 ros2_control 硬件接口的核心。
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "script_sender_port",
            default_value="50002",
            description="The driver will offer an interface to query the external_control URScript on this port.",
            # 此端口用于ROS驱动发送和管理在机器人控制器上运行的URScript（external_control.urscript）。驱动程序通过它来上传、验证和查询这个核心控制脚本的状态。
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "trajectory_port",
            default_value="50003",
            description="Port that will be opened for trajectory control.",
            # 这是轨迹命令端口。当 ros2_control（例如 joint_trajectory_controller）计算出机器人要执行的运动轨迹点时，它会通过这个端口将这些指令发送给机器人上运行的URScript。
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "script_command_port",
            default_value="50004",
            description="Port that will be opened to forward URScript commands to the robot.",
            # 这是辅助脚本命令端口。它允许ROS系统发送额外、非实时的URScript命令到机器人，而不会中断主 ros2_control 的实时控制。
            # 例如，你可以通过这个端口发送一个URScript命令来控制一个通过机器人I/O连接的夹爪（set_digital_out(1, True)），而不需要停止机器人的轨迹运动。
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])