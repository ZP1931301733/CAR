# Car_Navigation Project Instructions

此功能包用于实现四轮四转式机器人基于激光雷达和GPS的二维地图导航，目前实现在gazebo仿真环境下

仿真环境控制器依赖于[ros_control](https://github.com/ros-controls/ros_control) 包

# 安装ros_control
```
sudo apt update
sudo apt install ros-${ROS_DISTRO}-ros-control ros-${ROS_DISTRO}-ros-controllers
```
`ROS_DISTRO`是你的ROS版本，比如：
- Melodic: `ros-melodic-ros-control`
- Noetic: `ros-noetic-ros-control`
