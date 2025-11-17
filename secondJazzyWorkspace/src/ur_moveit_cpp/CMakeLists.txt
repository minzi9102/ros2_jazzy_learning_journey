cmake_minimum_required(VERSION 3.8)
project(ur_moveit_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 必需的构建工具
find_package(ament_cmake REQUIRED)

# 主要的 C++ 依赖 (来自 <depend>)
find_package(rclcpp REQUIRED)
find_package(controller_manager REQUIRED)
find_package(ur_robot_driver REQUIRED)
find_package(ur_description REQUIRED)
find_package(rviz2 REQUIRED)
find_package(ur_client_library REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(moveit REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(tf2_eigen REQUIRED)

# 消息依赖 (来自 <depend> 和 <exec_depend>)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(ur_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# === 1. 添加你的可执行文件 ===
add_executable(ur_moveit_cpp src/my_moveit_cpp.cpp)
# add_executable(ur_moveit_cpp src/moveit_cpp_tutorial.cpp)

# === 2. 链接所有依赖 ===
# ament_target_dependencies 会自动处理头文件包含和库链接
ament_target_dependencies(ur_moveit_cpp
  rclcpp
  controller_manager
  ur_robot_driver
  ur_description
  rviz2
  ur_client_library
  tf2_ros
  rviz_visual_tools
  moveit
  moveit_visual_tools
  geometry_msgs
  moveit_msgs
  shape_msgs
  ur_msgs
  control_msgs
  trajectory_msgs
  tf2_eigen
)

# === 3. 安装可执行文件 ===
# 这样你才能使用 ros2 run
install(TARGETS
  ur_moveit_cpp
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
