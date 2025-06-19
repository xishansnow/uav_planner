# uav_planner

本工程基于 ROS 团队给出的 ROS_Gazebo 联合开发模板（https://github.com/gazebosim/ros_gz_project_template）。其中：

ros_gz_uav_description - 保存 Gazebo 仿真系统所需的世界文件、仿真所需资源。

ros_gz_uav_gazebo - 保存仅与 Gazebo 端有关的代码和配置。

ros_gz_uav_application - 保存与仅与 ros2 端有关的代码和配置。

ros_gz_uav_bringup - 保存 launch 文件和其他高层工具，可能会涉及到 ROS 与 Gazebo 之间的转换代理等

目前的开发主要在 ROS 部分，因此都在 ros_gz_uav_application 包中。