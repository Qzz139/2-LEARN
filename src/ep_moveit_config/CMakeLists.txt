# 安装 MoveIt 规划、关节限速和控制器配置，供 Launch 按包路径加载。
cmake_minimum_required(VERSION 3.5)
project(ep_moveit_config)
find_package(ament_cmake REQUIRED)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
ament_package()
