# 安装上游模型、网格及启动文件，同时保留模型资产许可证。
cmake_minimum_required(VERSION 3.5)
project(robomaster_description)
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf launch meshes DESTINATION share/${PROJECT_NAME})
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})
ament_package()
