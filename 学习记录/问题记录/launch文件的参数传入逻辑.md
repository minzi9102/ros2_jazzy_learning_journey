# launch文件参数传入逻辑详解

作者：Minzi

---

## 第一种：generate_launch_description() + launch_setup(context)类

所有`launch.py`文件都会在运行后直接执行generate_launch_description()函数，因此，所有运行逻辑都会从这个函数中的内容开始

**参数传入逻辑：“命令行覆盖”机制**

**声明 (Declaration)**： 在Python启动文件（.py）中，generate_launch_description 函数里的 DeclareLaunchArgument("ur_type", ...) 这一行，是在“声明”：“我这个启动文件可以接受一个名叫 `ur_type` 的参数。”

**默认值 (Default)**： DeclareLaunchArgument 里的 default_value="..."（如果存在）设置了一个默认值。如果用户在启动时不提供这个参数，系统就会使用这个默认值。

**覆盖 (Override)**： 当你在 ros2 launch 命令最后附加 ur_type:=ur3 时，你是在提供一个**“命令行覆盖”**。

- ROS 2 启动系统在运行 generate_launch_description 函数之前，会先解析你的整个 ros2 launch 命令。它会把你附加的所有 xxx:=yyy 参数（例如 ur_type:=ur3）全部存储在一个临时的“覆盖字典”里。

**匹配与替换 (Matching & Replacement)**： 当 generate_launch_description 函数被执行并遇到 DeclareLaunchArgument("ur_type", ...) 时，启动系统会执行这个逻辑：

    “这个脚本声明了参数 ur_type。”

    “我去查一下我的‘覆盖字典’里有没有 ur_type？”

    “哈，找到了！用户在命令行里指定了 ur_type:=ur3。”

    “那么，DeclareLaunchArgument 中为 ur_type 设置的任何 default_value 都将被忽略。这个参数的最终值被确定为 "ur3"。”

- 如果启动系统在“覆盖字典”里没找到 ur_type，它才会去使用 DeclareLaunchArgument 里的 default_value。如果既没有命令行覆盖，也没有 default_value（如此前的 robot_ip），启动系统就会报错，因为一个必需的参数没有被提供。

**传入时间：启动流程四步走**

- **时刻 1**：【命令执行时】 你按下回车键执行 `ros2 launch ... ur_type:=ur3`。 ROS 2 的 launch 工具首先解析命令行，它立即识别到 `ur_type:=ur3` 是一个覆盖参数，并将其（以及其他所有 := 参数）存入内存。

- **时刻 2**：【声明阶段 (运行 generate_launch_description)】 launch 工具开始执行 `generate_launch_description` 函数。 当执行到 `DeclareLaunchArgument("ur_type", ...)` 时，启动系统执行上述的“匹配与替换”逻辑。 在这一刻，系统内部就确定了 `ur_type` 的最终值是 "`ur3`"。 `generate_launch_description` 函数执行完毕，返回了包含 `OpaqueFunction(function=launch_setup)` 的 `LaunchDescription`。

- **时刻 3**：【上下文创建 `(Context Creation)`】 在真正开始“执行”任何节点之前，启动系统会创建一个**LaunchContext 对象**。 这个 `context` 对象就像一个“全局配置单”，它持有了所有参数的最终值（例如：`ur_type = "ur3"`, `robot_ip = "192.168.1.100"` 等）。

- **时刻 4**：【执行阶段 (运行 `launch_setup`)】 启动系统现在开始执行 `LaunchDescription` 里的动作。它遇到了 `OpaqueFunction(function=launch_setup)`。 `OpaqueFunction` 的机制就是：

    1. 调用你指定的 launch_setup 函数。

    2. 自动将那个充满了最终值的 LaunchContext 对象（在时刻 3 创建的）作为第一个参数传递进去。

这就是 def launch_setup(context): 中的 context 的由来。

---

## 第二种：def generate_launch_description() + def declare_arguments()类

“定义所有节点，并把参数占位符传给它们，让启动系统在最后一刻自动替换这些占位符”

- 这个文件不使用 OpaqueFunction，它的逻辑分为两个主要阶段：

    1. 设置阶段 (Setup Phase)：generate_launch_description 函数被同步执行，用于创建所有动作（Action）和节点的“蓝图”或“配方”。

    2. 执行阶段 (Execution Phase)：ROS 2 启动系统接收这个“蓝图”并开始异步执行它，处理事件和解析占位符。

- 阶段一：设置阶段 (执行 generate_launch_description 函数)

    - 当你在终端运行 ros2 launch ... 时，ROS 2 启动系统会立即调用这个文件中的 generate_launch_description() 函数。以下是函数内部的执行顺序：

        1. 创建“占位符”： 创建 LaunchConfiguration 对象（launch_rviz, ur_type, warehouse_sqlite_path 等）。

        - 关键：此时，ur_type 不是字符串 "ur5e"。它是一个占位符对象，代表“将来要从启动上下文中获取的 ur_type 的值”。

        2. 准备“配方”：

        - 这个对象现在是一个完整的“配方集”，它包含了所有生成最终配置（URDF、SRDF、YAMLs）所需的信息和占位符。

        4. 创建主启动描述 (LD)：

        - ld = LaunchDescription() 创建一个空的“动作”列表。

        5. 声明与验证参数：

        - declare_arguments() 函数被调用。

        - 它返回一个包含所有 DeclareLaunchArgument 对象的列表。

        - ld.add_entity(...) 将这些声明添加到 ld 中。

            - 关键时刻：在这一步，ROS 2 启动系统会检查你从命令行传入的参数（例如 ur_type:=ur5e），将其与 DeclareLaunchArgument 声明进行匹配和验证（例如检查 choices）。

        - 在这一刻，ur_type 的“最终值” ("ur5e") 被锁定并存储在启动系统的内部上下文（Context）中，准备在“执行阶段”使用。

        6. 定义“守卫”节点：

        - wait_robot_description = Node(...) 创建“守卫”节点的定义。

        - ld.add_action(wait_robot_description) 将这个节点添加到 ld 的根级动作列表。这意味着它将在“执行阶段”立即启动。

        8. 定义“延迟”节点：

        - move_group_node = Node(...) 被执行。

        - 它创建节点的定义。

        - 它的 parameters 列表被填入 moveit_config.to_dict() 和 warehouse_ros_config。

            - 关键：这些参数仍然是“配方”（包含占位符），而不是最终的字符串。

        - servo_node = Node(...) 和 rviz_node = Node(...) 也以相同的方式被定义（不是启动）。

        9. 定义“事件处理器”：

        - RegisterEventHandler(...) 被执行。

        - 它创建了一个事件处理器定义。

        - OnProcessExit(...) 指定了这个处理器的规则：

            - 监听 (target_action)：wait_robot_description 节点。

            - 动作 (on_exit)：启动 [move_group_node, rviz_node, servo_node] 列表中的所有节点。

        - ld.add_action(...) 将这个事件处理器添加到 ld 的根级动作列表。

        10. 返回“蓝图”：

        - generate_launch_description 函数执行完毕。

        - 它返回 ld (LaunchDescription) 对象。

        - 这个 ld 对象现在是一个完整的“启动蓝图”，它告诉 ROS 2 启动系统：

            - “立即启动 wait_robot_description 节点。”

            - “立即激活一个事件处理器，监听 wait_robot_description 的退出。”

- 阶段二：执行阶段 (启动系统执行“蓝图”)

    现在，ROS 2 启动系统拿到了 ld “蓝图”并开始真正地执行它。

    11. 启动“守卫”：

        - 启动系统找到 wait_robot_description 节点（来自步骤 6）并立即启动它。

        - 启动系统激活事件处理器（来自步骤 9），该处理器现在开始监听 wait_robot_description 进程。

    12. 等待...：

        - （假设你的另一个 ur_control.launch.py 也在运行）

        - wait_robot_description 节点在后台运行，它不断检查 /robot_description 话题。

        - 几秒钟后，robot_state_publisher (由 ur_control.launch.py 启动) 成功发布了 URDF。

        - wait_robot_description 节点检测到了话题，成功退出（返回码 0）。

    13. 事件触发：

       -  事件处理器（来自步骤 11）捕获到 wait_robot_description 的退出事件。

        - 它执行其 on_exit 动作，向启动系统发出一个新命令：“请立即启动这三个节点：move_group_node, rviz_node, servo_node。”

    14. 启动 move_group_node (占位符替换的时刻)：

        - 启动系统准备启动 move_group_node（定义来自步骤 8）。

        - 它首先检查条件（Condition）：没有条件，继续执行。

        - 它查看 parameters 列表，发现里面全是“配方”。

        - 开始解析“配方”：

            - 它解析 moveit_config.robot_description_semantic。

            - 这个“配方”说：“我需要 ur_type 占位符的值。”

            - 启动系统查询它在步骤 5 中锁定的内部上下文，取出字符串 "ur5e"。

            - “配方”继续：“好的，现在请用 "ur5e" 作为 name 参数，去运行 xacro 处理 ur.srdf.xacro 文件。”

            - xacro 在一个子进程中被执行，并返回最终的 SRDF 文本字符串。

            - 这个过程对所有参数（URDF、kinematics.yaml 等）重复执行。

        - 最终启动：move_group 进程被启动，并被传入所有已完全解析、不含任何占位符的最终参数。

    15. 启动 rviz_node：

        - 启动系统准备启动 rviz_node（定义来自步骤 8）。

        - 检查条件：它检查 condition=IfCondition(launch_rviz)。

        - 它解析 launch_rviz 占位符，从上下文中获取其值（默认为 "true"）。

        - IfCondition("true") 评估为 True，继续执行。

        - 它以与步骤 14 相同的方式，解析 rviz_node 的所有参数（包括 moveit_config 的“配方”）。

        - 最终启动：rviz2 进程被启动，并传入所有最终参数。

    16. 启动 servo_node：

        - 启动系统准备启动 servo_node（定义来自步骤 8）。

        - 检查条件：它检查 condition=IfCondition(launch_servo)。

        - 它解析 launch_servo 占位符，从上下文中获取其值（默认为 "false"）。

        - IfCondition("false") 评估为 False。

        - 最终启动：该节点被跳过，不会被启动。

    17. 完成：

        - move_group_node 和 rviz_node 正在运行。

        - wait_robot_description 已经退出。

        - servo_node 被跳过。

        - 启动过程完成。