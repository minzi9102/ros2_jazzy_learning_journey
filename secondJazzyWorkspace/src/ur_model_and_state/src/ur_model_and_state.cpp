// MoveIt头文件包含
#include <moveit/robot_model_loader/robot_model_loader.hpp>//-> 需要moveit_ros_planning
#include <moveit/robot_model/robot_model.hpp>//-> 需要moveit_core
#include <moveit/robot_state/robot_state.hpp>//-> 需要moveit_core

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  //允许参数传入的代码
  /*
  说明: 在ROS 2中，节点默认情况下不接受未在代码中显式声明（declare_parameter()）的参数。

  设置 automatically_declare_parameters_from_overrides(true) 就像是给节点开了一个“后门”，
  它告诉节点：“如果launch文件试图设置一个我没有声明的参数（比如 robot_description），请接受它。”

  没有这一行，即使你的launch文件正确设置了 robot_description，这个C++节点也会忽略它，导致 RobotModelLoader 失败。
  */
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("ur_model_and_state", node_options);
  const auto& LOGGER = node->get_logger();

  
  robot_model_loader::RobotModelLoader robot_model_loader(node);// 1. 机器人模型加载->需要传入URDF参数（robot_description）
  //真正“需要”这些参数的代码是 RobotModelLoader 类的内部实现。
  //说明：当您创建 robot_model_loader::RobotModelLoader 对象并（隐式或显式地）调用 .getModel() 时，
  // 这个类会自动在它所关联的ROS 2节点 (node) 上查找名为 robot_description 和 robot_description_semantic 的参数。
  // 如果找不到这些参数，getModel() 将失败，程序会崩溃或出错。

  //robot_description: 这是机器人的URDF (Unified Robot Description Format) 内容，
  // 通常是一个XML字符串。它定义了机器人的连杆、关节和物理属性。

  //robot_description_semantic: 这是机器人的SRDF (Semantic Robot Description Format) 内容，也是一个XML字符串。
  // 它定义了机器人的"语义"信息，例如关节组 (Joint Model Groups，比如代码中用到的 "ur_manipulator")、末端执行器、默认位姿等。

  const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
  RCLCPP_INFO(LOGGER, "模型框架: %s", kinematic_model->getModelFrame().c_str());

  // 使用RobotModel，我们可以构造一个RobotState，
  // 用于维护机器人的配置。我们将把所有关节设置为
  // 默认值。然后我们可以获取JointModelGroup，
  // 它代表特定组的机器人模型，例如UR机器人的"ur_manipulator"。
  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(kinematic_model));
  robot_state->setToDefaultValues();
  const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("ur_manipulator");
  //从SRDF参数(robot_description_semantic)中提取

  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

  // 获取关节值
  // ^^^^^^^^^^
  // 我们可以获取存储在状态中的当前关节值集。
  std::vector<double> joint_values;
  robot_state->copyJointGroupPositions(joint_model_group, joint_values);
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    RCLCPP_INFO(LOGGER, "关节 %s: %f", joint_names[i].c_str(), joint_values[i]);
  }

  // 关节限制
  // ^^^^^^^^
  // setJointGroupPositions()本身不强制执行关节限制，
  // 但调用enforceBounds()会执行此操作。
  /* 将机械臂的一个关节设置在其关节限制之外 */
  joint_values[0] = 5.57;
  robot_state->setJointGroupPositions(joint_model_group, joint_values);

  /* 检查是否有关节超出其关节限制 */
  RCLCPP_INFO_STREAM(LOGGER, "当前状态是 " << (robot_state->satisfiesBounds() ? "有效的" : "无效的"));

  /* 强制执行关节限制并再次检查 */
  robot_state->enforceBounds();
  RCLCPP_INFO_STREAM(LOGGER, "当前状态是 " << (robot_state->satisfiesBounds() ? "有效的" : "无效的"));

  // 正向运动学
  // ^^^^^^^^^^
  // 现在，我们可以为一组随机关节值计算正向运动学。
  // 注意，我们想要找到"tool0"的位姿，它是机器人
  // "ur_manipulator"组中最远端的连杆。
  robot_state->setToRandomPositions(joint_model_group);
  const Eigen::Isometry3d& end_effector_state = robot_state->getGlobalLinkTransform("tool0");

  /* 打印末端执行器位姿。记住这是在模型框架中的 */
  RCLCPP_INFO_STREAM(LOGGER, "平移: \n" << end_effector_state.translation() << "\n");
  RCLCPP_INFO_STREAM(LOGGER, "旋转: \n" << end_effector_state.rotation() << "\n");

  // 逆运动学
  // ^^^^^^^^
  // 现在我们可以为UR机器人求解逆运动学(IK)。
  // 要求解IK，我们需要以下内容：
  //
  // * 末端执行器期望的位姿（默认是"ur_manipulator"链中的最后一个连杆）：
  //   我们在上面计算的end_effector_state。
  // * 超时时间：0.1秒
  double timeout = 0.1;
  bool found_ik = robot_state->setFromIK(joint_model_group, end_effector_state, timeout);

  // 现在，我们可以打印出IK解（如果找到的话）：
  if (found_ik)
  {
    robot_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
      RCLCPP_INFO(LOGGER, "关节 %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
  }
  else
  {
    RCLCPP_INFO(LOGGER, "未找到IK解");
  }

  // 获取雅可比矩阵
  // ^^^^^^^^^^^^^^
  // 我们还可以从RobotState获取雅可比矩阵
  Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
  Eigen::MatrixXd jacobian;
  robot_state->getJacobian(joint_model_group, robot_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                           reference_point_position, jacobian);
  RCLCPP_INFO_STREAM(LOGGER, "雅可比矩阵: \n" << jacobian << "\n");
  // 教程结束

  rclcpp::shutdown();
  return 0;
}