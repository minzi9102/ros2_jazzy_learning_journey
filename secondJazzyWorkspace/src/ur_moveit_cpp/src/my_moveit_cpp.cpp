#include <rclcpp/rclcpp.hpp> //<depend>rclcpp</depend>
#include <memory>
#include <thread>
#include <string> // 增加 <string> 头文件

// MoveitCpp 相关头文件
#include <moveit/moveit_cpp/moveit_cpp.hpp>       //<depend>moveit</depend>
#include <moveit/moveit_cpp/planning_component.hpp>

// 消息类型头文件
#include <geometry_msgs/msg/point_stamped.hpp>    //<depend>geometry_msgs</depend>

// 可视化工具
#include <moveit_visual_tools/moveit_visual_tools.h> //<depend>moveit_visual_tools</depend>

#include <Eigen/Geometry> // 增加 Eigen 头文件

// #include <tf2_eigen/tf2_eigen.h> // (新增) 用于 Eigen 和 ROS 消息转换 <depend>tf2_eigen</depend>
#include <tf2_eigen/tf2_eigen.hpp> // (新增) 用于 Eigen 和 ROS 消息转换 <depend>tf2_eigen</depend>

namespace rvt = rviz_visual_tools;

// 按照 ROS 日志记录的最佳实践，定义一个文件作用域内的静态 LOGGER
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_cpp_tutorial");

int main(int argc, char** argv)
{
  // 初始化 ROS 2 C++ 客户端库
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  RCLCPP_INFO(LOGGER, "初始化节点");

  // 允许节点加载未声明的参数
  // 注意：最佳实践是应在相应的类中显式声明参数，并提供其用途描述
  node_options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("run_moveit_cpp", "", node_options);

  // 启动一个单线程执行器 (SingleThreadedExecutor)
  // 用于当前状态监视器 (CurrentStateMonitor) 获取机器人的状态信息
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  // 在一个分离的线程中运行执行器，使其不阻塞主线程
  std::thread([&executor]() { executor.spin(); }).detach();

  // --- 设置 ---

  // 定义规划组的名称
  static const std::string PLANNING_GROUP = "ur_manipulator";
  // 定义日志记录器的名称
  static const std::string LOGNAME = "moveit_cpp_tutorial";
  // 定义要使用的控制器列表 (来自 ros2_controllers)
  static const std::vector<std::string> CONTROLLERS(1, "scaled_joint_trajectory_controller");

  /* * 短暂休眠1秒
   * 确保 MoveIt 能够接收到当前的机器人状态（joint_states）
   * 否则，机器人模型可能处于所有关节为零的默认状态
   */
  rclcpp::sleep_for(std::chrono::seconds(1));

  RCLCPP_INFO(LOGGER, "启动 MoveIt C++ 接口...");

  // 初始化 MoveItCpp 实例
  auto moveit_cpp_ptr = std::make_shared<moveit_cpp::MoveItCpp>(node);
  // 启动规划场景监视器 (PlanningSceneMonitor) 并提供规划场景服务
  moveit_cpp_ptr->getPlanningSceneMonitorNonConst()->providePlanningSceneService();

  // 初始化规划组件 (PlanningComponent)，指定规划组
  auto planning_components = std::make_shared<moveit_cpp::PlanningComponent>(PLANNING_GROUP, moveit_cpp_ptr);
  // 获取机器人模型
  auto robot_model_ptr = moveit_cpp_ptr->getRobotModel();
  // 获取机器人的起始状态
  auto robot_start_state = planning_components->getStartState();
  // 获取规划组的关节模型组 (JointModelGroup)
  auto joint_model_group_ptr = robot_model_ptr->getJointModelGroup(PLANNING_GROUP);

  // --- 可视化 ---

  // MoveItVisualTools 提供了在 RViz 中可视化对象、机器人和轨迹的能力
  // 以及逐步调试脚本等工具
  moveit_visual_tools::MoveItVisualTools visual_tools(node, "base_link", "moveit_cpp_tutorial",
                                                    moveit_cpp_ptr->getPlanningSceneMonitorNonConst());
  // 清除 RViz 中的所有历史标记
  visual_tools.deleteAllMarkers();
  // 加载远程控制（例如，RViz GUI 插件中的 "Next" 按钮）
  visual_tools.loadRemoteControl();

  // 设置文本显示的位姿（z轴上移1.0米）
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  // 在 RViz 中发布文本
  visual_tools.publishText(text_pose, "MoveItCpp_Demo", rvt::WHITE, rvt::XLARGE);
  // 触发可视化更新
  visual_tools.trigger();

  // --- 开始执行 ---
  
  // 提示用户在 RViz 界面中点击 "next" 来开始
  visual_tools.prompt("在 RvizVisualToolsGui 窗口中点击 'next' 开始");

  // --- 规划到目标状态 ---
  // 将规划的起始状态明确设置为机器人当前的状态
  planning_components->setStartStateToCurrentState();

  // 设置规划目标
  // planning_components->setGoal(target_state); // 可以使用 RobotState 对象
  planning_components->setGoal("test_configuration"); // 或者使用在 SRDF 中定义的命名位姿 (Named Target)

  // 从设置的起始状态规划到目标状态
  auto plan_solution3 = planning_components->plan();

  // 检查规划是否成功
  if (plan_solution3)
  {
    // 可视化规划结果
    moveit::core::RobotState robot_state(robot_model_ptr);
    // 将规划解决方案中的起始状态消息 (RobotState msg) 转换为 RobotState 对象
    moveit::core::robotStateMsgToRobotState(plan_solution3.start_state, robot_state);

    // 在 RViz 中显示起始位姿的坐标轴
    visual_tools.publishAxisLabeled(robot_state.getGlobalLinkTransform("tool0"), "start_pose");
    // visual_tools.publishAxisLabeled(target_joints_vec, "target_pose"); // (可视化目标位姿 - 可选)
    visual_tools.publishText(text_pose, "plan1:test_configuration", rvt::WHITE, rvt::XLARGE);
    // visual_tools.publishTrajectoryLine(plan_solution3.trajectory, joint_model_group_ptr); // (可视化轨迹线 - 可选)
    visual_tools.trigger();

    /* * 执行轨迹
     * 如果需要在真实（或模拟）机器人上执行规划好的轨迹，请取消以下注释
     */
    /* bool blocking = true; */
    RCLCPP_INFO(LOGGER, "执行轨迹 1: 运动到 'test_configuration'");
    moveit_cpp_ptr->execute(plan_solution3.trajectory, CONTROLLERS);
  }
    else
  {
    RCLCPP_ERROR(LOGGER, "规划 1: 'test_configuration' 失败");
  }

  // 提示用户继续
  visual_tools.prompt("在 RvizVisualToolsGui 窗口中点击 'next' 继续");
  // 清除所有标记
  visual_tools.deleteAllMarkers();
  visual_tools.trigger();
  RCLCPP_INFO(LOGGER, "规划 2: 沿 X 轴正向平移 0.1 米");

  // 将规划的起始状态明确设置为机器人当前的状态
  planning_components->setStartStateToCurrentState();
  // 获取当前的机器人状态
  auto start_state_4 = planning_components->getStartState();
  if (!start_state_4)
  {
    RCLCPP_ERROR(LOGGER, "获取规划 2 的起始状态失败");
  }
  else
  {
    // 1. 获取 "tool0" 连杆的当前位姿 (Eigen::Isometry3d 格式)
    Eigen::Isometry3d current_tool0_pose = start_state_4->getGlobalLinkTransform("tool0");
    RCLCPP_INFO(LOGGER, "当前 'tool0' X 坐标: %f", current_tool0_pose.translation().x());

    // 2. 创建目标位姿，基于当前位姿
    Eigen::Isometry3d target_tool0_pose = current_tool0_pose;
    // 沿 Y 轴平移 0.3 米
    target_tool0_pose.translation().x() += 0.1;
    target_tool0_pose.translation().y() -= 0.1;
    target_tool0_pose.translation().z() -= 0.1;
    RCLCPP_INFO(LOGGER, "目标 'tool0' X 坐标: %f", target_tool0_pose.translation().x());

    // 3. 设置笛卡尔空间位姿目标
    // 3a. 创建一个 PoseStamped 消息
    geometry_msgs::msg::PoseStamped target_pose_stamped;
    target_pose_stamped.header.frame_id = "base_link"; // 假设在 "base_link" 坐标系中
    target_pose_stamped.header.stamp = node->get_clock()->now();
    // 3b. 使用 tf2_eigen 将 Eigen::Isometry3d 转换为 geometry_msgs::msg::Pose
    target_pose_stamped.pose = tf2::toMsg(target_tool0_pose);

    planning_components->setGoal(target_pose_stamped, "tool0");

    // 4. 规划
    auto plan_solution4 = planning_components->plan();

    // 5. 检查和执行
    if (plan_solution4)
    {
      RCLCPP_INFO(LOGGER, "X 轴平移规划成功，准备执行");
      
      // 可视化
      visual_tools.publishAxisLabeled(current_tool0_pose, "start_pose_2");
      visual_tools.publishAxisLabeled(target_tool0_pose, "target_pose_2");
      visual_tools.publishText(text_pose, "plan2", rvt::WHITE, rvt::XLARGE);
      // visual_tools.publishTrajectoryLine(plan_solution4.trajectory, joint_model_group_ptr);
      visual_tools.trigger();

      // --- 执行轨迹 2 ---
      moveit_cpp_ptr->execute(plan_solution4.trajectory, CONTROLLERS);
    }
    else
    {
      RCLCPP_ERROR(LOGGER, "规划 2: Y 轴平移规划失败");
    }
  
  }


  // 提示用户继续
  visual_tools.prompt("在 RvizVisualToolsGui 窗口中点击 'next' 继续");
  // 清除所有标记
  visual_tools.deleteAllMarkers();
  visual_tools.trigger();

  // 将规划的起始状态明确设置为机器人当前的状态
  planning_components->setStartStateToCurrentState();

  // 设置规划目标
  // planning_components->setGoal(target_state); // 可以使用 RobotState 对象
  planning_components->setGoal("up"); // 或者使用在 SRDF 中定义的命名位姿 (Named Target)

  // 从设置的起始状态规划到目标状态
  auto plan_solution5 = planning_components->plan();

  // 检查规划是否成功
  if (plan_solution5)
  {
    // 可视化规划结果
    moveit::core::RobotState robot_state(robot_model_ptr);
    // 将规划解决方案中的起始状态消息 (RobotState msg) 转换为 RobotState 对象
    moveit::core::robotStateMsgToRobotState(plan_solution3.start_state, robot_state);

    // 在 RViz 中显示起始位姿的坐标轴
    visual_tools.publishAxisLabeled(robot_state.getGlobalLinkTransform("tool0"), "start_pose");
    // visual_tools.publishAxisLabeled(target_joints_vec, "target_pose"); // (可视化目标位姿 - 可选)
    visual_tools.publishText(text_pose, "moveit::core::RobotState_Goal_Pose", rvt::WHITE, rvt::XLARGE);
    // visual_tools.publishTrajectoryLine(plan_solution3.trajectory, joint_model_group_ptr); // (可视化轨迹线 - 可选)
    visual_tools.trigger();

    /*执行轨迹 */
    moveit_cpp_ptr->execute(plan_solution5.trajectory, CONTROLLERS);
  }

  

  // 提示用户结束
  visual_tools.prompt("在 RvizVisualToolsGui 窗口中点击 'next' 结束");
  visual_tools.deleteAllMarkers();
  visual_tools.trigger();

  RCLCPP_INFO(LOGGER, "关闭节点");
  // 关闭 ROS 2
  rclcpp::shutdown();
  return 0;
}